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
/// Sub-id: master handoff notification `E9 02 side` (0 = left becomes master, 1 = right).
pub(crate) const SUB_HANDOFF: u8 = 0x02;
/// Sub-id: firmware build of one half. Request `08 E9 03 side`, reply `E9 03 side len text…`.
pub(crate) const SUB_FIRMWARE: u8 = 0x03;
/// Longest build string carried in a packet (`v0.2.0-21-g4bf2fa45` fits; a `-dirty` suffix is cut).
pub(crate) const FIRMWARE_BUILD_MAX: usize = 24;
/// This half's build id (`git describe` at build time). `cargo:rustc-env` from the keyboard's
/// build script is visible only in the keyboard crate, so the generated `main` hands it over.
static FIRMWARE_BUILD: Mutex<RawMutex, core::cell::Cell<&'static str>> = Mutex::new(core::cell::Cell::new(""));

pub fn set_firmware_build(build: &'static str) {
    FIRMWARE_BUILD.lock(|c| c.set(build));
}

pub(crate) fn firmware_build() -> &'static str {
    FIRMWARE_BUILD.lock(|c| c.get())
}
/// `fw_layout` bit telling the host that the right half is the central (dynamic master).
pub(crate) const FW_LAYOUT_CENTRAL_RIGHT: u8 = 0x80;
/// `fw_layout` bit telling the host that the other half is linked to this one.
pub(crate) const FW_LAYOUT_PEER_LINKED: u8 = 0x40;
/// Longest subscription a host may ask for.
pub(crate) const MAX_TTL_SECS: u8 = 120;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LayerState {
    pub active: u8,
    pub default: u8,
    pub mask: u16,
    pub fw_layout: u8,
    /// The other half is connected to this one (the helper dims a half that is not).
    pub peer_linked: bool,
}

#[derive(Clone, Copy)]
struct Subscription {
    transport: HostTransport,
    deadline_ms: u64,
    /// Last state pushed to this subscriber; identical states are not repeated.
    last_sent: Option<LayerState>,
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
    out[6] = state.fw_layout & 0x3F;
    if state.peer_linked {
        out[6] |= FW_LAYOUT_PEER_LINKED;
    }
    #[cfg(all(feature = "split", feature = "_ble"))]
    if !crate::split::role::central_is_left_now() {
        out[6] |= FW_LAYOUT_CENTRAL_RIGHT;
    }
    out
}

/// `E9 03 side len text…`; the text is cut to `FIRMWARE_BUILD_MAX` bytes.
pub(crate) fn encode_firmware(side_is_right: bool, build: &[u8]) -> [u8; 32] {
    let mut out = [0u8; 32];
    out[0] = NAMESPACE;
    out[1] = SUB_FIRMWARE;
    out[2] = side_is_right as u8;
    let n = build.len().min(FIRMWARE_BUILD_MAX);
    out[3] = n as u8;
    out[4..4 + n].copy_from_slice(&build[..n]);
    out
}

/// This half's build as a fixed, zero-padded array (what the split link carries to the other half).
pub(crate) fn firmware_build_bytes() -> [u8; FIRMWARE_BUILD_MAX] {
    let mut out = [0u8; FIRMWARE_BUILD_MAX];
    let b = firmware_build().as_bytes();
    let n = b.len().min(FIRMWARE_BUILD_MAX);
    out[..n].copy_from_slice(&b[..n]);
    out
}

/// The build string stored for one half, as sent over the link: text up to the first zero byte.
pub(crate) fn build_text(bytes: &[u8]) -> &[u8] {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    &bytes[..end]
}

/// `E9 02 side`, rest zero.
pub(crate) fn encode_handoff(new_central_is_left: bool) -> [u8; 32] {
    let mut out = [0u8; 32];
    out[0] = NAMESPACE;
    out[1] = SUB_HANDOFF;
    out[2] = (!new_central_is_left) as u8;
    out
}

/// Tell the subscribed host that the halves are swapping roles; the sender resets right after.
pub(crate) fn notify_handoff(new_central_is_left: bool, now_ms: u64) {
    if let Some(transport) = active_subscriber(now_ms) {
        try_send_host_reply(transport, encode_handoff(new_central_is_left));
    }
}

/// Subscribe or renew for `ttl_secs` (clamped to `MAX_TTL_SECS`); `0` unsubscribes.
pub(crate) fn subscribe(transport: HostTransport, ttl_secs: u8, now_ms: u64) {
    let sub = (ttl_secs > 0).then(|| Subscription {
        transport,
        deadline_ms: now_ms + u64::from(ttl_secs.min(MAX_TTL_SECS)) * 1_000,
        last_sent: None,
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

/// Send `state` to the subscriber, if any and if it differs from the last state sent to it.
/// Non-blocking; dropped when the reply queue is full.
pub(crate) fn notify(state: LayerState, now_ms: u64) {
    let transport = SUBSCRIPTION.lock(|cell| {
        let mut slot = cell.borrow_mut();
        match slot.as_mut() {
            Some(sub) if now_ms < sub.deadline_ms => {
                if sub.last_sent == Some(state) {
                    None
                } else {
                    sub.last_sent = Some(state);
                    Some(sub.transport)
                }
            }
            Some(_) => {
                *slot = None;
                None
            }
            None => None,
        }
    });
    if let Some(transport) = transport {
        try_send_host_reply(transport, encode_state(state));
    }
}

/// Whether the other half is linked; `true` when this firmware is not a BLE split (nothing to miss).
pub(crate) fn peer_linked_now() -> bool {
    #[cfg(all(feature = "split", feature = "_ble"))]
    {
        crate::split::role::peer_linked()
    }
    #[cfg(not(all(feature = "split", feature = "_ble")))]
    {
        true
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
    fn encode_state_reports_the_other_half_as_linked_or_offline() {
        let state = LayerState { active: 0, default: 0, mask: 1, fw_layout: 0, peer_linked: true };
        assert_eq!(encode_state(state)[6] & FW_LAYOUT_PEER_LINKED, FW_LAYOUT_PEER_LINKED);
        let offline = LayerState { peer_linked: false, ..state };
        assert_eq!(encode_state(offline)[6] & FW_LAYOUT_PEER_LINKED, 0);
        // The layout id keeps its own bits below the two status flags.
        let with_layout = LayerState { fw_layout: 3, peer_linked: true, ..state };
        assert_eq!(encode_state(with_layout)[6] & 0x3F, 3);
    }

    #[test]
    fn a_link_change_alone_is_a_new_state_for_the_dedupe() {
        // The helper must learn about an offline half at once, not at the next layer change.
        let state = LayerState { active: 0, default: 0, mask: 1, fw_layout: 0, peer_linked: true };
        assert_ne!(state, LayerState { peer_linked: false, ..state });
    }

    #[test]
    fn encode_firmware_carries_side_length_and_text() {
        let p = encode_firmware(true, b"v0.2.0-21-g4bf2fa45");
        assert_eq!(&p[..4], &[0xE9, 0x03, 1, 19]);
        assert_eq!(&p[4..23], b"v0.2.0-21-g4bf2fa45");
        assert!(p[23..].iter().all(|b| *b == 0));
        let long = encode_firmware(false, &[b'x'; 40]);
        assert_eq!(long[3] as usize, FIRMWARE_BUILD_MAX);
        let none = encode_firmware(false, b"");
        assert_eq!(&none[..4], &[0xE9, 0x03, 0, 0]);
    }

    #[test]
    fn build_text_stops_at_the_padding() {
        let mut bytes = [0u8; FIRMWARE_BUILD_MAX];
        bytes[..5].copy_from_slice(b"v1.2.");
        assert_eq!(build_text(&bytes), b"v1.2.");
        assert_eq!(build_text(&[0u8; 4]), b"");
    }

    #[test]
    fn encode_handoff_layout() {
        assert_eq!(&encode_handoff(false)[..3], &[0xE9, 0x02, 1]);
        assert_eq!(&encode_handoff(true)[..3], &[0xE9, 0x02, 0]);
        assert!(encode_handoff(true)[3..].iter().all(|b| *b == 0));
    }

    #[test]
    fn encode_state_sets_central_right_bit_when_right_is_central() {
        use crate::split::role::{Role, Side, init};
        let state = LayerState { active: 1, default: 0, mask: 0b10, fw_layout: 1, peer_linked: true };
        init(Side::Right, Role::Central);
        let linked = 1 | FW_LAYOUT_PEER_LINKED;
        assert_eq!(encode_state(state)[6], linked | FW_LAYOUT_CENTRAL_RIGHT);
        init(Side::Left, Role::Central);
        assert_eq!(encode_state(state)[6], linked);
        init(Side::Right, Role::Peripheral);
        assert_eq!(encode_state(state)[6], linked, "left is central");
    }

    #[test]
    fn encode_state_matches_spec_layout() {
        let bytes = encode_state(LayerState {
            active: 3,
            default: 0,
            mask: 0b0000_1110,
            fw_layout: 1,
            peer_linked: true,
        });
        assert_eq!(&bytes[..7], &[0xE9, 0x01, 3, 0, 0x0E, 0x00, 1 | FW_LAYOUT_PEER_LINKED]);
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

    /// Build a 1x1 keymap with four layers (so the tri-layer rule applies) and run `f` against it.
    fn with_four_layer_keymap(f: impl FnOnce(&crate::keymap::KeyMap)) {
        use crate::config::{BehaviorConfig, PositionalConfig};
        use crate::keymap::{KeyMap, KeymapData};
        use rmk_types::action::KeyAction;

        let mut data = KeymapData::new([[[KeyAction::No]]; 4]);
        let mut behavior = BehaviorConfig::default();
        let positional = PositionalConfig::<1, 1>::default();
        let keymap = embassy_futures::block_on(KeyMap::new(&mut data, &mut behavior, &positional));
        f(&keymap);
    }

    fn drain_usb_replies() {
        while crate::channel::HOST_USB_REPLY.try_receive().is_ok() {}
    }

    #[test]
    fn layer_mask_reports_all_active_layers() {
        with_four_layer_keymap(|keymap| {
            keymap.activate_layer(1);
            keymap.activate_layer(2);
            keymap.update_fn_layer_state(); // what keyboard.rs does after MO(1)/MO(2): layer 3 = 1 && 2
            assert_eq!(keymap.active_layer(), 3);
            assert_eq!(keymap.layer_mask(), 0b0000_1110);
        });
    }

    #[test]
    fn no_subscriber_sends_nothing() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        subscribe(HostTransport::Usb, 0, 0);
        notify(
            LayerState {
                active: 1,
                default: 0,
                mask: 0b10,
                fw_layout: 0,
            peer_linked: true,
        },
            0,
        );
        assert!(crate::channel::HOST_USB_REPLY.try_receive().is_err());
    }

    #[test]
    fn subscribed_layer_change_is_sent_on_usb() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        subscribe(HostTransport::Usb, 60, 0);
        notify(
            LayerState {
                active: 2,
                default: 0,
                mask: 0b100,
                fw_layout: 1,
            peer_linked: true,
        },
            10,
        );
        let sent = crate::channel::HOST_USB_REPLY.try_receive().expect("one reply queued");
        assert_eq!(&sent[..7], &[0xE9, 0x01, 2, 0, 0x04, 0x00, 1 | FW_LAYOUT_PEER_LINKED]);
    }

    #[test]
    fn full_reply_queue_drops_notification() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        subscribe(HostTransport::Usb, 60, 0);
        while crate::channel::HOST_USB_REPLY.try_send([0u8; 32]).is_ok() {}
        notify(
            LayerState {
                active: 1,
                default: 0,
                mask: 0b10,
                fw_layout: 0,
            peer_linked: true,
        },
            0,
        ); // must not panic
        assert_eq!(
            active_subscriber(0),
            Some(HostTransport::Usb),
            "dropping a packet does not unsubscribe"
        );
        drain_usb_replies();
    }

    #[test]
    fn keymap_layer_change_notifies_subscriber() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        subscribe(HostTransport::Usb, 60, 0);
        with_four_layer_keymap(|keymap| {
            keymap.activate_layer(1);
            let sent = crate::channel::HOST_USB_REPLY.try_receive().expect("layer change pushed");
            assert_eq!(&sent[..6], &[0xE9, 0x01, 1, 0, 0x02, 0x00]);
        });
    }

    #[test]
    fn unchanged_state_is_sent_once() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        subscribe(HostTransport::Usb, 60, 0);
        let state = LayerState {
            active: 0,
            default: 0,
            mask: 0,
            fw_layout: 0,
            peer_linked: true,
        };
        notify(state, 0);
        notify(state, 1);
        notify(state, 2);
        assert!(crate::channel::HOST_USB_REPLY.try_receive().is_ok(), "first state is sent");
        assert!(crate::channel::HOST_USB_REPLY.try_receive().is_err(), "identical repeats are not");
    }

    #[test]
    fn fresh_subscription_resends_current_state_on_next_change() {
        let _g = test_lock().lock().unwrap();
        drain_usb_replies();
        let state = LayerState {
            active: 1,
            default: 0,
            mask: 0b10,
            fw_layout: 0,
            peer_linked: true,
        };
        subscribe(HostTransport::Usb, 60, 0);
        notify(state, 0);
        drain_usb_replies();
        subscribe(HostTransport::Usb, 60, 1); // renew or new subscriber: dedupe memory resets
        notify(state, 2);
        assert!(crate::channel::HOST_USB_REPLY.try_receive().is_ok());
    }

    #[test]
    fn ttl_is_clamped_to_max() {
        let _g = test_lock().lock().unwrap();
        subscribe(HostTransport::Usb, 255, 0);
        assert_eq!(active_subscriber(120_000 - 1), Some(HostTransport::Usb));
        assert_eq!(active_subscriber(120_000), None);
    }
}
