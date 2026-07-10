# psoxide developer entry points.
#
# `make verify` runs the Verus formal-verification proofs. Verus is installed
# out-of-band (it is NOT a Cargo dependency); if no `verus` binary is
# discoverable the target prints a message and skips (exit 0), so it never
# breaks a machine without Verus.

.PHONY: verify

# Verus is located via (in order): the VERUS env var, the VERUS_BIN env var,
# or a `verus` on PATH.
VERUS ?= $(VERUS_BIN)
VERUS := $(if $(VERUS),$(VERUS),verus)

# The four hand-mirrored Verus proof files (lib.rs is the cargo marker, not a
# Verus spec, so it is intentionally excluded).
PROOF_FILES := \
	crates/psoxide-proof/src/bus_map.rs \
	crates/psoxide-proof/src/decode.rs \
	crates/psoxide-proof/src/map_region.rs \
	crates/psoxide-proof/src/timing.rs

verify:
	@if ! command -v "$(VERUS)" >/dev/null 2>&1; then \
		echo "Verus not found (looked for VERUS / VERUS_BIN / 'verus' on PATH), skipping verification."; \
		exit 0; \
	fi; \
	for f in $(PROOF_FILES); do \
		echo "[verus] checking $$f"; \
		"$(VERUS)" "$$f" || exit $$?; \
	done; \
	echo "All Verus checks passed."
