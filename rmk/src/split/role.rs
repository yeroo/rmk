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
/// A peripheral takes over only after its own enumeration, and the link that carries the central's
/// synced status, have both been stable this long (both halves enumerating together after a PC
/// boot must not race).
pub const TAKE_OVER_SETTLE_MS: u64 = 1_000;
/// Without any link (other half off), a peripheral on a USB host takes over after this long.
pub const NO_LINK_TAKE_OVER_MS: u64 = 3_000;
/// Non-default central powered but never enumerated for this long hands back (cable moved to a
/// wall adapter, a port that keeps 5 V with no host).
pub const NO_HOST_GRACE_MS: u64 = 30_000;
/// Default central stuck as a peripheral with no link and no host for this long reclaims its role
/// (both halves ended up peripheral after a lost handoff).
pub const RESUME_DEFAULT_MS: u64 = 30_000;

/// Whether the central's synced connection status says it has a USB host (a suspended host is
/// still a host: it wakes on the next key).
pub fn central_status_has_host(usb: rmk_types::connection::UsbState) -> bool {
    matches!(usb, rmk_types::connection::UsbState::Configured | rmk_types::connection::UsbState::Suspended)
}

/// What a half does with an incoming handoff: its new role, and whether it must first tell the
/// host (only the central has a subscribed helper).
pub fn handoff_received_plan(side: Side, role: Role, new_central_is_left: bool) -> (Role, bool) {
    (role_after_handoff(side, new_central_is_left), role == Role::Central)
}

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
    /// Non-default central without USB power or host: become peripheral (handoff, then reset).
    HandBack,
    /// Central that saw its peer claim mastership: become peripheral silently.
    Yield,
    /// Default central that has been a linkless, hostless peripheral too long: reset as central.
    ResumeDefault,
}

/// Start of a condition that has held continuously (`None` while it does not hold).
#[derive(Default)]
struct Since(Option<u64>);

impl Since {
    /// Record the condition for this sample; returns how long it has held (0 on its first sample).
    fn update(&mut self, holds: bool, now_ms: u64) -> Option<u64> {
        if holds {
            let since = *self.0.get_or_insert(now_ms);
            Some(now_ms.saturating_sub(since))
        } else {
            self.0 = None;
            None
        }
    }
}

pub struct RoleMachine {
    side: Side,
    role: Role,
    vbus_lost: Since,
    usb_configured: Since,
    usb_unconfigured: Since,
    link_up: Since,
    link_down: Since,
}

impl RoleMachine {
    pub fn new(side: Side, role: Role) -> Self {
        Self {
            side,
            role,
            vbus_lost: Since::default(),
            usb_configured: Since::default(),
            usb_unconfigured: Since::default(),
            link_up: Since::default(),
            link_down: Since::default(),
        }
    }

    pub fn step(&mut self, inputs: RoleInputs) -> Decision {
        let now = inputs.now_ms;
        let configured_for = self.usb_configured.update(inputs.local_usb_configured, now);
        let unconfigured_for = self.usb_unconfigured.update(!inputs.local_usb_configured, now);
        let link_up_for = self.link_up.update(inputs.link_up, now);
        let link_down_for = self.link_down.update(!inputs.link_up, now);
        let is_default = self.role == default_role(self.side);
        match self.role {
            Role::Peripheral => {
                if inputs.central_has_usb_host {
                    return Decision::Stay; // first master stays
                }
                match (configured_for, link_up_for, link_down_for) {
                    // Linked: wait for our enumeration and the link (hence the synced status) to settle.
                    (Some(c), Some(l), _) if c >= TAKE_OVER_SETTLE_MS && l >= TAKE_OVER_SETTLE_MS => Decision::TakeOver,
                    // No link at all: the other half may be off; take over on our own after a while.
                    (Some(c), None, _) if c >= NO_LINK_TAKE_OVER_MS => Decision::TakeOver,
                    // Both halves peripheral (lost handoff): the default central reclaims its role.
                    (None, None, Some(d)) if !is_default && default_role(self.side) == Role::Central && d >= RESUME_DEFAULT_MS => {
                        Decision::ResumeDefault
                    }
                    _ => Decision::Stay,
                }
            }
            Role::Central => {
                if inputs.peer_claim_seen && !inputs.local_usb_configured {
                    return Decision::Yield;
                }
                let vbus_lost_for = self.vbus_lost.update(!inputs.vbus, now);
                if is_default {
                    // The default central is the Bluetooth master without a cable.
                    return Decision::Stay;
                }
                if vbus_lost_for.is_some_and(|t| t >= USB_LOSS_GRACE_MS)
                    || unconfigured_for.is_some_and(|t| t >= NO_HOST_GRACE_MS)
                {
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
/// `init` ran: this is a dual-role build. Other keyboards keep their build-time constants.
static DYNAMIC: AtomicBool = AtomicBool::new(false);
/// Peripheral role: the central's synced status says it has a USB host (cleared with the link).
pub static CENTRAL_HAS_USB_HOST: AtomicBool = AtomicBool::new(false);
/// This half's own USB device reached `Configured`; maintained by the USB device handler.
/// Distinct from `state::current_usb_state()`, which a peripheral overwrites with the
/// central's synced status.
pub static LOCAL_USB_CONFIGURED: AtomicBool = AtomicBool::new(false);
/// Ask the split loop to send `RoleHandoff { new_central_is_left }` to the other half.
pub static ROLE_HANDOFF_TX: Signal<crate::RawMutex, bool> = Signal::new();
/// A master-claim advertisement from the stored peer address was seen (central role).
pub static PEER_CLAIM_SEEN: Signal<crate::RawMutex, ()> = Signal::new();
/// Peripheral role: the split link to the central is up (maintained by `split::ble::peripheral`).
pub static PERIPHERAL_LINK_UP: AtomicBool = AtomicBool::new(false);
/// The other half's address as the central knows it; only a claim from this address counts.
static PEER_ADDRESS: embassy_sync::blocking_mutex::Mutex<crate::RawMutex, core::cell::Cell<Option<[u8; 6]>>> =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(None));

pub fn set_peer_address(addr: [u8; 6]) {
    PEER_ADDRESS.lock(|c| c.set(Some(addr)));
}

pub fn peer_address() -> Option<[u8; 6]> {
    PEER_ADDRESS.lock(|c| c.get())
}

pub fn init(side: Side, role: Role) {
    CURRENT_SIDE_IS_LEFT.store(side == Side::Left, Ordering::Relaxed);
    CURRENT_ROLE_IS_CENTRAL.store(role == Role::Central, Ordering::Relaxed);
    DYNAMIC.store(true, Ordering::Relaxed);
}

/// Whether this firmware runs the dynamic master logic at all.
pub fn dynamic_active() -> bool {
    DYNAMIC.load(Ordering::Relaxed)
}

/// Peripheral role: the split link went up or down (a down link also forgets the central's status).
pub fn set_peripheral_link(up: bool) {
    PERIPHERAL_LINK_UP.store(up, Ordering::Relaxed);
    if !up {
        CENTRAL_HAS_USB_HOST.store(false, Ordering::Relaxed);
    }
}

pub fn current_side() -> Side {
    if CURRENT_SIDE_IS_LEFT.load(Ordering::Relaxed) { Side::Left } else { Side::Right }
}

pub fn current_role() -> Role {
    if CURRENT_ROLE_IS_CENTRAL.load(Ordering::Relaxed) { Role::Central } else { Role::Peripheral }
}

/// Whether the left half is the central right now; the build-time constant for keyboards that
/// do not use dynamic roles.
pub fn central_is_left_now() -> bool {
    if dynamic_active() {
        central_is_left(current_side(), current_role())
    } else {
        crate::SPLIT_CENTRAL_IS_LEFT
    }
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

/// The other half announced the new central: become what that leaves for this side. A central
/// tells its host first (the helper shows "switching master").
pub async fn apply_handoff_received(new_central_is_left: bool) -> ! {
    let (role, announce) = handoff_received_plan(current_side(), current_role(), new_central_is_left);
    info!("role: handoff received, new central is {}", if new_central_is_left { "left" } else { "right" });
    #[cfg(feature = "_ble")]
    if announce {
        crate::split::role_task::announce_handoff(new_central_is_left).await;
    }
    #[cfg(not(feature = "_ble"))]
    let _ = announce;
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
    fn synced_central_status_counts_configured_and_suspended_as_a_host() {
        use rmk_types::connection::UsbState;
        assert!(central_status_has_host(UsbState::Configured));
        assert!(central_status_has_host(UsbState::Suspended));
        assert!(!central_status_has_host(UsbState::Enabled));
        assert!(!central_status_has_host(UsbState::Disabled));
    }

    #[test]
    fn take_over_waits_for_a_stable_enumeration_and_a_fresh_link() {
        // I4: both halves enumerate together after a PC boot; the first poll must not decide.
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        assert_eq!(m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(0) }), Decision::Stay);
        assert_eq!(m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(500) }), Decision::Stay);
        assert_eq!(
            m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(TAKE_OVER_SETTLE_MS) }),
            Decision::TakeOver
        );
        // A link that has just come up has not synced the central's status yet: wait for it.
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        m.step(RoleInputs { link_up: false, local_usb_configured: true, vbus: true, ..inputs(0) });
        m.step(RoleInputs { link_up: false, local_usb_configured: true, vbus: true, ..inputs(TAKE_OVER_SETTLE_MS) });
        // link comes up now
        assert_eq!(
            m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(TAKE_OVER_SETTLE_MS + 250) }),
            Decision::Stay
        );
        assert_eq!(
            m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(2 * TAKE_OVER_SETTLE_MS + 250) }),
            Decision::TakeOver
        );
    }

    #[test]
    fn take_over_without_a_link_after_the_settle_time() {
        // The other half is off: no synced status will ever come; take over anyway.
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        for t in (0..TAKE_OVER_SETTLE_MS).step_by(250) {
            assert_eq!(m.step(RoleInputs { link_up: false, local_usb_configured: true, vbus: true, ..inputs(t) }), Decision::Stay);
        }
        assert_eq!(
            m.step(RoleInputs { link_up: false, local_usb_configured: true, vbus: true, ..inputs(NO_LINK_TAKE_OVER_MS) }),
            Decision::TakeOver
        );
    }

    #[test]
    fn default_central_as_lonely_peripheral_resumes_its_default_role() {
        // I5: both halves ended up peripheral; the left reclaims the central role after a while.
        let mut m = RoleMachine::new(Side::Left, Role::Peripheral);
        for t in (0..RESUME_DEFAULT_MS).step_by(1_000) {
            assert_eq!(m.step(RoleInputs { link_up: false, ..inputs(t) }), Decision::Stay);
        }
        assert_eq!(m.step(RoleInputs { link_up: false, ..inputs(RESUME_DEFAULT_MS) }), Decision::ResumeDefault);
        // A linked peripheral or the right half never does this.
        let mut m = RoleMachine::new(Side::Left, Role::Peripheral);
        assert_eq!(m.step(RoleInputs { link_up: true, ..inputs(RESUME_DEFAULT_MS * 2) }), Decision::Stay);
        let mut m = RoleMachine::new(Side::Right, Role::Peripheral);
        assert_eq!(m.step(RoleInputs { link_up: false, ..inputs(RESUME_DEFAULT_MS * 2) }), Decision::Stay);
    }

    #[test]
    fn right_central_powered_without_a_host_hands_back_after_the_long_grace() {
        // I6: cable moved to a wall adapter in under 5 s, or a port that keeps 5 V with no host.
        let mut m = RoleMachine::new(Side::Right, Role::Central);
        for t in (0..NO_HOST_GRACE_MS).step_by(1_000) {
            assert_eq!(m.step(RoleInputs { vbus: true, local_usb_configured: false, ..inputs(t) }), Decision::Stay);
        }
        assert_eq!(
            m.step(RoleInputs { vbus: true, local_usb_configured: false, ..inputs(NO_HOST_GRACE_MS) }),
            Decision::HandBack
        );
        // A host that is merely suspended keeps the local device configured: no hand-back.
        let mut m = RoleMachine::new(Side::Right, Role::Central);
        for t in (0..NO_HOST_GRACE_MS * 2).step_by(5_000) {
            assert_eq!(m.step(RoleInputs { vbus: true, local_usb_configured: true, ..inputs(t) }), Decision::Stay);
        }
    }

    #[test]
    fn handoff_received_by_the_central_announces_it_to_the_host() {
        // I1: the old master is the half with the subscribed helper; it must send E9 02 before resetting.
        assert_eq!(handoff_received_plan(Side::Left, Role::Central, false), (Role::Peripheral, true));
        assert_eq!(handoff_received_plan(Side::Right, Role::Peripheral, false), (Role::Central, false));
        assert_eq!(handoff_received_plan(Side::Right, Role::Central, true), (Role::Peripheral, true));
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
        assert_eq!(m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(0) }), Decision::Stay);
        assert_eq!(
            m.step(RoleInputs { local_usb_configured: true, vbus: true, ..inputs(TAKE_OVER_SETTLE_MS) }),
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
