//! Device side of the dynamic master half: samples the inputs of [`RoleMachine`] every
//! 250 ms and carries out its decisions (handoff over the link, then reset into the new role).

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};
use rmk_types::connection::UsbState;

use super::role::{
    Decision, HANDOFF_SETTLE_MS, LOCAL_USB_CONFIGURED, PEER_CLAIM_SEEN, PERIPHERAL_LINK_UP, ROLE_HANDOFF_TX, Role,
    RoleInputs, RoleMachine, Side, current_role, current_side, reset_into,
};

const POLL_MS: u64 = 250;

pub async fn run_role_task() -> ! {
    let side = current_side();
    let role = current_role();
    let mut machine = RoleMachine::new(side, role);
    info!("role: running as {:?} on the {:?} half", role, side);
    loop {
        Timer::after(Duration::from_millis(POLL_MS)).await;
        let link_up = match role {
            Role::Central => crate::split::ble::central::all_peripherals_connected(),
            Role::Peripheral => PERIPHERAL_LINK_UP.load(Ordering::Relaxed),
        };
        let inputs = RoleInputs {
            now_ms: embassy_time::Instant::now().as_millis(),
            local_usb_configured: LOCAL_USB_CONFIGURED.load(Ordering::Relaxed),
            vbus: vbus_present(),
            link_up,
            // Only meaningful on the peripheral, whose status is the central's synced copy.
            central_has_usb_host: role == Role::Peripheral && crate::state::current_usb_state() == UsbState::Configured,
            peer_claim_seen: PEER_CLAIM_SEEN.try_take().is_some(),
            ble_host_connected: false,
        };
        match machine.step(inputs) {
            Decision::Stay => {}
            Decision::TakeOver => handoff(side == Side::Left, link_up, Role::Central).await,
            Decision::HandBack => handoff(side != Side::Left, link_up, Role::Peripheral).await,
            Decision::Yield => {
                info!("role: yield to the peer's master claim");
                reset_into(Role::Peripheral)
            }
        }
    }
}

/// Tell the other half who the new central is (when the link can carry it), then reset.
async fn handoff(new_central_is_left: bool, link_up: bool, new_role: Role) -> ! {
    info!(
        "role: handoff, new central is the {} half (link up: {})",
        if new_central_is_left { "left" } else { "right" },
        link_up
    );
    if link_up {
        ROLE_HANDOFF_TX.signal(new_central_is_left);
        Timer::after_millis(HANDOFF_SETTLE_MS).await;
    }
    reset_into(new_role)
}

fn vbus_present() -> bool {
    #[cfg(feature = "_nrf_ble")]
    {
        embassy_nrf::pac::POWER.usbregstatus().read().vbusdetect()
    }
    #[cfg(not(feature = "_nrf_ble"))]
    {
        false
    }
}
