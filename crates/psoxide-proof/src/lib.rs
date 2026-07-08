//! Verus proof specs and lemmas for psoxide.
//!
//! A foundational scaffold crate hosting formal correctness proofs, checked
//! **out-of-band** with the `verus` toolchain (see `scripts/verus-check.ps1`).
//!
//! The proof source files under `src/` (e.g. `bus_map.rs`) use
//! `use vstd::prelude::*` and are **not** declared as modules here, so a normal
//! `cargo build` compiles only this trivial marker and stays green even without
//! Verus installed.

/// Returns a static marker string indicating the proof crate is initialized.
///
/// ## Examples
///
/// ```
/// # use psoxide_proof::proof_crate_marker;
/// assert_eq!(proof_crate_marker(), "proof-scaffold-ready");
/// ```
#[must_use]
pub fn proof_crate_marker() -> &'static str {
    "proof-scaffold-ready"
}
