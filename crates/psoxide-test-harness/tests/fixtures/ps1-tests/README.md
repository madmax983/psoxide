# Vendored `ps1-tests` CPU binaries (JaCzekanski)

These prebuilt PlayStation test executables and their expected-output logs are
vendored from **JaCzekanski/ps1-tests**, which is distributed under the **MIT
License** (see `LICENSE` in this directory).

* Upstream project: https://github.com/JaCzekanski/ps1-tests
* Prebuilt binaries: Releases → `build-158`
* CI-friendly mirror used to obtain these files (GitHub egress is blocked in the
  build environment): `https://archive.org/download/tests_202203/tests.zip`
  (zip SHA1 `bc9d5f910cd79f86ec703f198f0bf46a12253ab6`)

Only the four CPU test programs relevant to the psoxide CPU gate are vendored
(total ~90 KB). Each directory holds the test's `.exe` and the hardware-captured
expected output `psx.log`.

## Per-file SHA1

| File | SHA1 |
|------|------|
| `cpu/access-time/access-time.exe`             | `bf3e90089b7e8a1b92ca18f2f547b205bf595559` |
| `cpu/code-in-io/code-in-io.exe`               | `409ac92b8f77ed753a85076a926cfb37dd7431ff` |
| `cpu/cop/cop.exe`                             | `74bf58ae5237263ab2580dcc5558c3e75b8b53f5` |
| `cpu/io-access-bitwidth/io-access-bitwidth.exe` | `9b1c1e87b7969d7c64f2c61d6bda020ab014668d` |

## How psoxide uses them

`tests/ps1_tests.rs` sideloads each `.exe` through the [`Harness`] and drives it
with the BIOS TTY / `printf` / exception HLE plus the hardware timers. The gate
tests assert that each binary **loads, executes end-to-end, and reaches its known
progress markers** — proving the sideloader + HLE + timer path works against real
PlayStation test binaries.

They do **not** yet assert a full byte-for-byte match against `psx.log`. Each
suite still exercises hardware psoxide does not implement (see the blockers table
in `crates/psoxide-test-harness/README.md`): the BIOS exception-dispatch chain
that invokes program-registered handlers, instruction/data bus-error exceptions,
the coprocessor-unusable exception, cycle-accurate access timing, and the
JOY/SIO/SPU/CD-ROM/MDEC peripherals. The `psx.log` files are vendored so the
gates can be tightened to golden-diff as that hardware lands.
