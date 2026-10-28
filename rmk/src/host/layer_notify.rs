//! Push the active layer to a subscribed Vial host.
//!
//! The OP36 layout helper subscribes with a Vial packet and renews before the
//! TTL runs out; while a subscription is live every layer change is sent as an
//! unsolicited 32-byte packet on the transport the subscription came from.
//! Without a subscriber nothing is sent, so other Vial clients see stock
//! behaviour.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;

use crate::RawMutex;
use crate::channel::{HostTransport, try_send_host_reply};

/// Vial custom-value namespace byte owned by the layout helper (vendor uses 0xE8).
pub(crate) const NAMESPACE: u8 = 0xE9;
/// Sub-id: layer state (subscribe / get / notification).
pub(crate) const SUB_STATE: u8 = 0x01;
/// Longest subscription a host may ask for.
pub(crate) const MAX_TTL_SECS: u8 = 120;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LayerState {
    pub active: u8,
    pub default: u8,
    pub mask: u16,
    pub fw_layout: u8,
}

#[derive(Clone, Copy)]
struct Subscription {
    transport: HostTransport,
    deadline_ms: u64,
}

static SUBSCRIPTION: Mutex<RawMutex, RefCell<Option<Subscription>>> = Mutex::new(RefCell::new(None));

/// `E9 01 active default mask_lo mask_hi fw_layout`, rest zero.
pub(crate) fn encode_state(state: LayerState) -> [u8; 32] {
    let mut out = [0u8; 32];
    out[0] = NAMESPACE;
    out[1] = SUB_STATE;
    out[2] = state.active;
    out[3] = state.default;
    out[4..6].copy_from_slice(&state.mask.to_le_bytes());
    out[6] = state.fw_layout;
    out
}

/// Subscribe or renew for `ttl_secs` (clamped to `MAX_TTL_SECS`); `0` unsubscribes.
pub(crate) fn subscribe(transport: HostTransport, ttl_secs: u8, now_ms: u64) {
    let sub = (ttl_secs > 0).then(|| Subscription {
        transport,
        deadline_ms: now_ms + u64::from(ttl_secs.min(MAX_TTL_SECS)) * 1_000,
    });
    SUBSCRIPTION.lock(|cell| *cell.borrow_mut() = sub);
}

/// The live subscriber's transport, clearing an expired subscription on the way.
pub(crate) fn active_subscriber(now_ms: u64) -> Option<HostTransport> {
    SUBSCRIPTION.lock(|cell| {
        let mut slot = cell.borrow_mut();
        match *slot {
            Some(sub) if now_ms < sub.deadline_ms => Some(sub.transport),
            Some(_) => {
                *slot = None;
                None
            }
            None => None,
        }
    })
}

/// Send `state` to the subscriber, if any. Non-blocking; dropped when the reply queue is full.
pub(crate) fn notify(state: LayerState, now_ms: u64) {
    if let Some(transport) = active_subscriber(now_ms) {
        try_send_host_reply(transport, encode_state(state));
    }
}

/// Milliseconds since boot; fixed at `0` in host tests so dispatcher tests are deterministic.
pub(crate) fn now_ms() -> u64 {
    #[cfg(test)]
    {
        0
    }
    #[cfg(not(test))]
    {
        embassy_time::Instant::now().as_millis()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Mutex, OnceLock};

    use super::*;
    use crate::channel::HostTransport;

    fn test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    #[test]
    fn encode_state_matches_spec_layout() {
        let bytes = encode_state(LayerState {
            active: 3,
            default: 0,
            mask: 0b0000_1110,
            fw_layout: 1,
        });
        assert_eq!(&bytes[..7], &[0xE9, 0x01, 3, 0, 0x0E, 0x00, 1]);
        assert!(bytes[7..].iter().all(|b| *b == 0));
    }

    #[test]
    fn subscription_is_active_until_deadline() {
        let _g = test_lock().lock().unwrap();
        subscribe(HostTransport::Usb, 60, 1_000);
        assert_eq!(active_subscriber(60_999), Some(HostTransport::Usb));
        assert_eq!(active_subscriber(61_000), None);
    }

    #[test]
    fn expired_subscription_is_dropped() {
        let _g = test_lock().lock().unwrap();
        subscribe(HostTransport::Usb, 1, 0);
        assert_eq!(active_subscriber(1_000), None);
        assert_eq!(
            active_subscriber(0),
            None,
            "expiry clears the store, it is not time-reversible"
        );
    }

    #[test]
    fn ttl_zero_unsubscribes() {
        let _g = test_lock().lock().unwrap();
        subscribe(HostTransport::Usb, 60, 0);
        subscribe(HostTransport::Usb, 0, 0);
        assert_eq!(active_subscriber(1), None);
    }

    #[test]
    fn ttl_is_clamped_to_max() {
        let _g = test_lock().lock().unwrap();
        subscribe(HostTransport::Usb, 255, 0);
        assert_eq!(active_subscriber(120_000 - 1), Some(HostTransport::Usb));
        assert_eq!(active_subscriber(120_000), None);
    }
}
