//! Which half is the master: the pure decision logic of the dynamic master half.
//!
//! Each half's firmware can run as the split central (master, talks to the host) or as the
//! peripheral. The rule is "the cable is the master": a half that a USB host enumerates takes
//! the central role; a non-default central that loses USB power hands it back; a central that
//! sees its peer claim mastership over the air yields. Everything here is host-testable; the
//! device-side plumbing (reset register, signals) lives in `role_task`.

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
