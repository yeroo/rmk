//! Dual-role ("dynamic master") firmware: one binary per half that can run as the split
//! central or as the peripheral, chosen at boot (`rmk::split::role`).
//!
//! `#[rmk_central(dynamic_role = "left")]` emits `pub mod as_central { pub async fn run(..) }`,
//! `#[rmk_peripheral(id = 0, dynamic_role = "left")]` emits `pub mod as_peripheral { .. }`, and
//! `rmk_dynamic_main!("left")` emits the shared interrupt bindings and the `main` that picks a
//! module. The right half's central view swaps the two matrix sections of `[split]` so its own
//! keys keep their row offset and the left half (now the peripheral) keeps offset 0.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use rmk_config::resolved::Hardware;
use rmk_config::resolved::hardware::{BoardConfig, SplitConfig};
use syn::ItemMod;

use crate::codegen::chip::bind_interrupt::expand_bind_interrupt;
use crate::codegen::feature::{get_rmk_features, is_feature_enabled};
use crate::codegen::keyboard_config::read_keyboard_toml_config;

/// The physical half a dual-role binary is built for (`dynamic_role = "left" | "right"`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum DynSide {
    Left,
    Right,
}

impl DynSide {
    pub(crate) fn parse(s: &str) -> Option<DynSide> {
        match s {
            "left" => Some(DynSide::Left),
            "right" => Some(DynSide::Right),
            _ => None,
        }
    }

    pub(crate) fn tokens(self) -> TokenStream2 {
        match self {
            DynSide::Left => quote! { ::rmk::split::role::Side::Left },
            DynSide::Right => quote! { ::rmk::split::role::Side::Right },
        }
    }
}

/// `[split]` as seen by this half when it runs as the central: the right half's own matrix
/// section becomes `central` (keeping its row offset) and the left section becomes the
/// peripheral it talks to.
pub(crate) fn central_split_view(split: &SplitConfig, side: DynSide) -> SplitConfig {
    match side {
        DynSide::Left => split.clone(),
        DynSide::Right => {
            let mut view = split.clone();
            view.central = split.peripheral[0].clone();
            view.peripheral[0] = split.central.clone();
            view
        }
    }
}

/// `[split]` as seen by this half when it runs as the peripheral (id 0).
pub(crate) fn peripheral_split_view(split: &SplitConfig, side: DynSide) -> SplitConfig {
    match side {
        DynSide::Right => split.clone(),
        DynSide::Left => {
            let mut view = split.clone();
            view.peripheral[0] = split.central.clone();
            view
        }
    }
}

fn with_split(hardware: &Hardware, split: SplitConfig) -> Hardware {
    let mut hw = hardware.clone();
    hw.board = BoardConfig::Split(split);
    hw
}

pub(crate) fn split_of(hardware: &Hardware) -> &SplitConfig {
    match &hardware.board {
        BoardConfig::Split(split) => split,
        _ => panic!("dynamic_role needs a `[split]` section in keyboard.toml"),
    }
}

pub(crate) fn central_hardware_view(hardware: &Hardware, side: DynSide) -> Hardware {
    with_split(hardware, central_split_view(split_of(hardware), side))
}

pub(crate) fn peripheral_hardware_view(hardware: &Hardware, side: DynSide) -> Hardware {
    with_split(hardware, peripheral_split_view(split_of(hardware), side))
}

/// Does this top-level item (a `bind_interrupts!` invocation or its `use`) define the shared
/// interrupt vector? Those must exist exactly once per binary.
fn is_bind_interrupts_item(item: &syn::Item) -> bool {
    match item {
        syn::Item::Macro(m) => m.mac.path.segments.last().is_some_and(|s| s.ident == "bind_interrupts"),
        syn::Item::Use(u) => quote!(#u).to_string().contains("bind_interrupts"),
        _ => false,
    }
}

fn filter_items(tokens: TokenStream2, keep: impl Fn(&syn::Item) -> bool) -> TokenStream2 {
    let file: syn::File = syn::parse2(tokens).expect("generated interrupt code is not a list of items");
    let items = file.items.into_iter().filter(|i| keep(i));
    quote! { #(#items)* }
}

/// Everything except the `bind_interrupts!` vector (helper fns, consts, tasks stay module-local).
pub(crate) fn strip_bind_interrupts(tokens: TokenStream2) -> TokenStream2 {
    filter_items(tokens, |i| !is_bind_interrupts_item(i))
}

/// Only the `bind_interrupts!` vector and its `use`.
pub(crate) fn only_bind_interrupts(tokens: TokenStream2) -> TokenStream2 {
    filter_items(tokens, is_bind_interrupts_item)
}

/// `pub mod <name> { use super::Irqs; <prelude> pub async fn run(spawner) { <body> } }`
pub(crate) fn wrap_role_module(name: &str, prelude: TokenStream2, body: TokenStream2) -> TokenStream2 {
    let name = quote::format_ident!("{}", name);
    quote! {
        pub mod #name {
            #[allow(unused_imports)]
            use super::Irqs;
            #prelude
            pub async fn run(spawner: ::embassy_executor::Spawner) {
                #body
            }
        }
    }
}

/// `rmk_dynamic_main!("left")`: the shared interrupt vector (the central's set is the superset:
/// USB, radio, clock) and the `main` that boots into `as_central` or `as_peripheral`.
pub(crate) fn expand_dynamic_main(side: DynSide) -> TokenStream2 {
    let keyboard_config = read_keyboard_toml_config();
    let hardware = keyboard_config
        .hardware()
        .expect("failed to resolve hardware config");
    let central_view = central_hardware_view(&hardware, side);
    let empty_mod: ItemMod = syn::parse_quote! { mod __rmk_dynamic_main {} };
    let irqs = only_bind_interrupts(expand_bind_interrupt(&central_view, &empty_mod));
    let side_tokens = side.tokens();
    // `RMK_FIRMWARE_BUILD` comes from this keyboard's build script (git describe); a build-script
    // env is visible only in the keyboard crate, so the generated main hands it to rmk.
    let set_build = if is_feature_enabled(&get_rmk_features(), "layer_notify") {
        quote! {
            ::rmk::host::layer_notify::set_firmware_build(option_env!("RMK_FIRMWARE_BUILD").unwrap_or(""));
        }
    } else {
        quote! {}
    };
    quote! {
        #irqs
        #[::embassy_executor::main]
        async fn main(spawner: ::embassy_executor::Spawner) {
            let side = #side_tokens;
            let role = ::rmk::split::role::read_boot_role().unwrap_or(::rmk::split::role::default_role(side));
            ::rmk::split::role::init(side, role);
            #set_build
            match role {
                ::rmk::split::role::Role::Central => as_central::run(spawner).await,
                ::rmk::split::role::Role::Peripheral => as_peripheral::run(spawner).await,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use rmk_config::resolved::hardware::{BoardConfig, SplitBoardConfig, SplitConfig};
    use rmk_config::MatrixConfig;

    use super::*;

    fn pins(prefix: &str) -> MatrixConfig {
        MatrixConfig {
            row_pins: Some(vec![format!("{prefix}_r0")]),
            col_pins: Some(vec![format!("{prefix}_c0")]),
            ..Default::default()
        }
    }

    fn split() -> SplitConfig {
        SplitConfig {
            connection: "ble".into(),
            central: SplitBoardConfig {
                rows: 3,
                cols: 6,
                row_offset: 0,
                col_offset: 0,
                matrix: pins("left"),
                ..Default::default()
            },
            peripheral: vec![SplitBoardConfig {
                rows: 3,
                cols: 6,
                row_offset: 3,
                col_offset: 0,
                matrix: pins("right"),
                ..Default::default()
            }],
        }
    }

    #[test]
    fn swapped_view_moves_right_pins_to_central_with_offset_3() {
        let v = central_split_view(&split(), DynSide::Right);
        assert_eq!(v.central.matrix.row_pins.as_deref(), Some(&["right_r0".to_string()][..]));
        assert_eq!(v.central.row_offset, 3);
        assert_eq!(v.peripheral[0].matrix.row_pins.as_deref(), Some(&["left_r0".to_string()][..]));
        assert_eq!(v.peripheral[0].row_offset, 0);
        let unchanged = central_split_view(&split(), DynSide::Left);
        assert_eq!(unchanged.central.row_offset, 0);
        assert_eq!(unchanged.central.matrix.row_pins.as_deref(), Some(&["left_r0".to_string()][..]));
    }

    #[test]
    fn left_as_peripheral_view_keeps_left_pins_with_offset_0() {
        let v = peripheral_split_view(&split(), DynSide::Left);
        assert_eq!(v.peripheral[0].matrix.row_pins.as_deref(), Some(&["left_r0".to_string()][..]));
        assert_eq!(v.peripheral[0].row_offset, 0);
        let unchanged = peripheral_split_view(&split(), DynSide::Right);
        assert_eq!(unchanged.peripheral[0].matrix.row_pins.as_deref(), Some(&["right_r0".to_string()][..]));
    }

    #[test]
    fn side_parses_from_the_attribute_string() {
        assert_eq!(DynSide::parse("left"), Some(DynSide::Left));
        assert_eq!(DynSide::parse("right"), Some(DynSide::Right));
        assert_eq!(DynSide::parse("middle"), None);
    }

    #[test]
    fn board_view_is_a_split_board() {
        let _ = BoardConfig::Split(split());
    }
}
