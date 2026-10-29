//! Which half is the master: the pure decision logic of the dynamic master half.
//!
//! Each half's firmware can run as the split central (master, talks to the host) or as the
//! peripheral. The rule is "the cable is the master": a half that a USB host enumerates takes
//! the central role; a non-default central that loses USB power hands it back; a central that
//! sees its peer claim mastership over the air yields. Everything here is host-testable; the
//! device-side plumbing (reset register, signals) lives in `role_task`.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::signal::Signal;

/// The role a half runs in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Central,
    Peripheral,
}

/// The physical half this binary was built for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// How long a non-default central tolerates missing USB power before handing the role back
/// (covers a KVM that cuts the port while switching hosts).
pub const USB_LOSS_GRACE_MS: u64 = 5_000;
/// Time left for the handoff message to leave the radio before the half resets.
pub const HANDOFF_SETTLE_MS: u64 = 300;

/// With no boot hint and no cable anywhere: the left half is the master.
pub fn default_role(side: Side) -> Role {
    match side {
        Side::Left => Role::Central,
        Side::Right => Role::Peripheral,
    }
}

/// Whether the left half is the central, given this half's side and role.
pub fn central_is_left(side: Side, role: Role) -> bool {
    matches!((side, role), (Side::Left, Role::Central) | (Side::Right, Role::Peripheral))
}

/// One observation of everything the decision depends on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RoleInputs {
    pub now_ms: u64,
    /// This half's own USB device reached `Configured` (a host enumerated it).
    pub local_usb_configured: bool,
    /// This half's `VBUSDETECT`.
    pub vbus: bool,
    /// The split link to the other half is connected.
    pub link_up: bool,
    /// Peripheral role only: the central's synced status says it has a USB host.
    pub central_has_usb_host: bool,
    /// Central role only: a master-claim advertisement from the stored peer address was seen.
    pub peer_claim_seen: bool,
    /// Central role only: informational, never changes a decision.
    pub ble_host_connected: bool,
}

/// What this half should do now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Stay,
    /// Become central (handoff over the link if it is up, then reset).
    TakeOver,
    /// Non-default central without USB power: become peripheral (handoff, then reset).
    HandBack,
    /// Central that saw its peer claim mastership: become peripheral silently.
    Yield,
}

pub struct RoleMachine {
    side: Side,
    role: Role,
    vbus_lost_at_ms: Option<u64>,
}

impl RoleMachine {
    pub fn new(side: Side, role: Role) -> Self {
        Self {
            side,
            role,
            vbus_lost_at_ms: None,
        }
    }

    pub fn step(&mut self, inputs: RoleInputs) -> Decision {
        match self.role {
            Role::Peripheral => {
                if inputs.local_usb_configured && !inputs.central_has_usb_host {
                    Decision::TakeOver
                } else {
                    Decision::Stay
                }
            }
            Role::Central => {
                if inputs.peer_claim_seen && !inputs.local_usb_configured {
                    return Decision::Yield;
                }
                if inputs.vbus {
                    self.vbus_lost_at_ms = None;
                    return Decision::Stay;
                }
                if self.role == default_role(self.side) {
                    // The default central is the Bluetooth master without a cable.
                    return Decision::Stay;
                }
                let lost_at = *self.vbus_lost_at_ms.get_or_insert(inputs.now_ms);
                if inputs.now_ms.saturating_sub(lost_at) >= USB_LOSS_GRACE_MS {
                    Decision::HandBack
                } else {
                    Decision::Stay
                }
            }
        }
    }
}

/// The role this half takes when the other half announces who the new central is.
pub fn role_after_handoff(side: Side, new_central_is_left: bool) -> Role {
    match (side, new_central_is_left) {
        (Side::Left, true) | (Side::Right, false) => Role::Central,
        _ => Role::Peripheral,
    }
}

/// `GPREGRET2` value that survives a soft reset and selects the role at the next boot.
/// (`GPREGRET` belongs to the Adafruit bootloader: `0x57` = enter DFU.)
pub fn magic_for(role: Role) -> u8 {
    match role {
        Role::Central => 0xC3,
        Role::Peripheral => 0xA5,
    }
}

pub fn role_from_magic(magic: u8) -> Option<Role> {
    match magic {
        0xC3 => Some(Role::Central),
        0xA5 => Some(Role::Peripheral),
        _ => None,
    }
}

// ---- runtime registry (set once at boot) ----

static CURRENT_SIDE_IS_LEFT: AtomicBool = AtomicBool::new(true);
static CURRENT_ROLE_IS_CENTRAL: AtomicBool = AtomicBool::new(true);
/// This half's own USB device reached `Configured`; maintained by the USB device handler.
/// Distinct from `state::current_usb_state()`, which a peripheral overwrites with the
/// central's synced status.
pub static LOCAL_USB_CONFIGURED: AtomicBool = AtomicBool::new(false);
/// Ask the split loop to send `RoleHandoff { new_central_is_left }` to the other half.
pub static ROLE_HANDOFF_TX: Signal<crate::RawMutex, bool> = Signal::new();
/// A master-claim advertisement from the stored peer address was seen (central role).
pub static PEER_CLAIM_SEEN: Signal<crate::RawMutex, ()> = Signal::new();

pub fn init(side: Side, role: Role) {
    CURRENT_SIDE_IS_LEFT.store(side == Side::Left, Ordering::Relaxed);
    CURRENT_ROLE_IS_CENTRAL.store(role == Role::Central, Ordering::Relaxed);
}

pub fn current_side() -> Side {
    if CURRENT_SIDE_IS_LEFT.load(Ordering::Relaxed) { Side::Left } else { Side::Right }
}

pub fn current_role() -> Role {
    if CURRENT_ROLE_IS_CENTRAL.load(Ordering::Relaxed) { Role::Central } else { Role::Peripheral }
}

/// Whether the left half is the central right now (replaces the build-time constant).
pub fn central_is_left_now() -> bool {
    central_is_left(current_side(), current_role())
}

/// The role hint left in `GPREGRET2` by the previous run, if any.
pub fn read_boot_role() -> Option<Role> {
    #[cfg(feature = "_nrf_ble")]
    {
        role_from_magic(embassy_nrf::pac::POWER.gpregret2().read().0 as u8)
    }
    #[cfg(not(feature = "_nrf_ble"))]
    {
        None
    }
}

/// Leave the role hint for the next boot and reset the chip.
pub fn reset_into(role: Role) -> ! {
    info!("role: resetting into {:?}", role);
    #[cfg(feature = "_nrf_ble")]
    embassy_nrf::pac::POWER
        .gpregret2()
        .write_value(embassy_nrf::pac::power::regs::Gpregret2(u32::from(magic_for(role))));
    #[cfg(all(target_arch = "arm", target_os = "none", any(target_abi = "eabi", target_abi = "eabihf")))]
    cortex_m::peripheral::SCB::sys_reset();
    #[cfg(not(all(target_arch = "arm", target_os = "none", any(target_abi = "eabi", target_abi = "eabihf"))))]
    panic!("reset_into is device-only");
}

/// The other half announced the new central: become what that leaves for this side.
pub async fn apply_handoff_received(new_central_is_left: bool) -> ! {
    let role = role_after_handoff(current_side(), new_central_is_left);
    info!("role: handoff received, new central is {}", if new_central_is_left { "left" } else { "right" });
    embassy_time::Timer::after_millis(HANDOFF_SETTLE_MS).await;
    reset_into(role)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(now_ms: u64) -> RoleInputs {
        RoleInputs {
            now_ms,
            local_usb_configured: false,
            vbus: false,
            link_up: true,
            central_has_usb_host: false,
            peer_claim_seen: false,
            ble_host_connected: false,
        }
    }

    #[test]
    fn handoff_received_maps_to_own_role() {
        assert_eq!(role_after_handoff(Side::Left, true), Role::Central);
        assert_eq!(role_after_handoff(Side::Left, false), Role::Peripheral);
        assert_eq!(role_after_handoff(Side::Right, true), Role::Peripheral);
        assert_eq!(role_after_handoff(Side::Right, false), Role::Central);
    }

    #[test]
    fn boot_role_magic_round_trips() {
        assert_eq!(role_from_magic(magic_for(Role::Central)), Some(Role::Central));
        assert_eq!(role_from_magic(magic_for(Role::Peripheral)), Some(Role::Peripheral));
        assert_eq!(magic_for(Role::Central), 0xC3);
        assert_eq!(magic_for(Role::Peripheral), 0xA5);
        assert_eq!(role_from_magic(0x00), None);
        assert_eq!(role_from_magic(0x57), None, "the bootloader's DFU magic is not a role");
    }

    #[test]
    fn defaults_are_left_central_right_peripheral() {
        assert_eq!(default_role(Side::Left), Role::Central);
        assert_eq!(default_role(Side::Right), Role::Peripheral);
        assert!(central_is_left(Side::Left, Role::Central));
        assert!(!central_is_left(Side::Right, Role::Central));
        assert!(central_is_left(Side::Right, Role::Peripheral));
    }

    #[test]
    fn peripheral_enumerated_by_host_takes_over() {
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        assert_eq!(
            m.step(RoleInputs {
                local_usb_configured: true,
                vbus: true,
                ..inputs(0)
            }),
            Decision::TakeOver
        );
    }

    #[test]
    fn vbus_without_host_never_takes_over() {
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        for t in (0..60_000).step_by(250) {
            assert_eq!(m.step(RoleInputs { vbus: true, ..inputs(t) }), Decision::Stay);
        }
    }

    #[test]
    fn peripheral_defers_when_central_has_usb_host() {
        let mut m = RoleMachine::new(Side::Left, Role::Peripheral);
        assert_eq!(
            m.step(RoleInputs {
                local_usb_configured: true,
                vbus: true,
                central_has_usb_host: true,
                ..inputs(0)
            }),
            Decision::Stay
        );
    }

    #[test]
    fn peripheral_ignores_synced_central_usb_state() {
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        assert_eq!(
            m.step(RoleInputs {
                central_has_usb_host: true,
                ..inputs(0)
            }),
            Decision::Stay
        );
    }

    #[test]
    fn right_central_hands_back_after_grace_without_vbus() {
        let mut m = RoleMachine::new(Side::Right, Role::Central);
        assert_eq!(m.step(RoleInputs { vbus: false, ..inputs(0) }), Decision::Stay);
        assert_eq!(m.step(RoleInputs { vbus: false, ..inputs(4_999) }), Decision::Stay);
        assert_eq!(m.step(RoleInputs { vbus: false, ..inputs(5_000) }), Decision::HandBack);
    }

    #[test]
    fn usb_loss_shorter_than_grace_is_ignored() {
        let mut m = RoleMachine::new(Side::Right, Role::Central);
        m.step(RoleInputs { vbus: false, ..inputs(0) });
        m.step(RoleInputs { vbus: true, ..inputs(2_000) });
        assert_eq!(
            m.step(RoleInputs { vbus: false, ..inputs(6_000) }),
            Decision::Stay,
            "timer restarted at 6 s"
        );
        assert_eq!(m.step(RoleInputs { vbus: false, ..inputs(11_000) }), Decision::HandBack);
    }

    #[test]
    fn left_central_without_vbus_stays_master_over_bluetooth() {
        let mut m = RoleMachine::new(Side::Left, Role::Central);
        for t in (0..120_000).step_by(1_000) {
            assert_eq!(
                m.step(RoleInputs {
                    vbus: false,
                    ble_host_connected: true,
                    ..inputs(t)
                }),
                Decision::Stay
            );
        }
    }

    #[test]
    fn central_yields_to_peer_claim_unless_on_usb_host() {
        let mut m = RoleMachine::new(Side::Left, Role::Central);
        assert_eq!(
            m.step(RoleInputs {
                peer_claim_seen: true,
                link_up: false,
                ble_host_connected: true,
                ..inputs(0)
            }),
            Decision::Yield
        );
        assert_eq!(
            m.step(RoleInputs {
                peer_claim_seen: true,
                link_up: false,
                local_usb_configured: true,
                vbus: true,
                ..inputs(1)
            }),
            Decision::Stay
        );
    }

    #[test]
    fn central_without_claim_never_yields() {
        let mut m = RoleMachine::new(Side::Left, Role::Central);
        for t in (0..600_000).step_by(5_000) {
            assert_eq!(
                m.step(RoleInputs {
                    link_up: false,
                    ble_host_connected: true,
                    ..inputs(t)
                }),
                Decision::Stay
            );
        }
    }
}
