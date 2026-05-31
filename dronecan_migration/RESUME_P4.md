# P4.0 resume guide — start here next session

Captures the build-wiring specifics worked out (and validated) while implementing P4.0, so a fresh session executes without re-deriving. Orientation order: `DESIGN.md` (locked architecture + the P4.0–P4.9 plan, §8) → this file (exact mechanics) → `PLAN.md` (status).

## Current state — branch `dronecan-libcanard`
- **4 submodules vendored** under `src/drivers/dronecan/`, **pinned to ArduPilot's commits**, **staged** (not committed unless a checkpoint commit was made): `libcanard 601ed35`, `dronecan_dsdlc de47a89`, `DSDL 04e0e81`, `pydronecan 08cda37`.
- **Codegen validated standalone** (rc=0): 206 headers / 176 sources.
- Everything else (docs, `.claude/` tooling) is **untracked** in the working tree. **If not committed, do not `git clean`.**

## Validated codegen command
```
python3 src/drivers/dronecan/dronecan_dsdlc/dronecan_dsdlc.py -O <outdir> \
  src/drivers/dronecan/DSDL/uavcan  src/drivers/dronecan/DSDL/dronecan \
  src/drivers/dronecan/DSDL/ardupilot src/drivers/dronecan/DSDL/com \
  src/drivers/dronecan/DSDL/cuav    src/drivers/dronecan/DSDL/mppt
```
→ `<outdir>/include/*.h` (206) + `<outdir>/src/*.c` (176). `pydronecan` is auto-found via the sibling `../pydronecan` path (the script inserts it on `sys.path`). Deps `empy`+`pexpect` are present in the PX4 env. Generated headers `#define` `_ID`/`_SIGNATURE`/`_MAX_SIZE`, declare `<type>_encode`(→byte length, 0=fail) and `<type>_decode`(→**TRUE on FAILURE**); the `cxx_iface` trait is `#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)` — **leave `DRONECAN_CXX_WRAPPERS` undefined** (that is the Q1 win).

## CMake wiring — mirror `src/drivers/cyphal/CMakeLists.txt`
- `px4_add_git_submodule(TARGET git_dronecan_libcanard PATH .../libcanard)` for each of the 4; `DEPENDS` on them in `px4_add_module`.
- **Configure-time generation** (like cyphal's `nnvg` at `cyphal/CMakeLists.txt:42-50`): `execute_process(COMMAND ${PYTHON_EXECUTABLE} <generator> -O ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated <6 DSDL dirs> RESULT_VARIABLE rc)` + `message(FATAL_ERROR)` on rc≠0. *(Not build-time — compiled `.c` must exist at configure for the glob; this is the deliberate deviation from DESIGN.md.)*
- `file(GLOB GEN_SRCS ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated/src/*.c)` → append to `SRCS`.
- `INCLUDES`: `${LIBCANARD_DIR}` (for `canard.h`) + `${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated/include`.
- `SRCS`: `${LIBCANARD_DIR}/canard.c` + `${GEN_SRCS}` + our files.
- `COMPILE_FLAGS`: `-DCANARD_ENABLE_TAO_OPTION=1 -DCANARD_ENABLE_CANFD=0 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_ALLOCATE_SEM=0` (NO `-DDRONECAN_CXX_WRAPPERS`).
- `set_source_files_properties(.../canard.c PROPERTIES COMPILE_FLAGS -Wno-cast-align)` (cyphal does this too).
- `px4_add_module(MODULE drivers__dronecan MAIN dronecan STACK_MAIN 4096 … MODULE_CONFIG module.yaml dronecan.params.yaml DEPENDS git_dronecan_* version)`.

## Kconfig — §2.1 "dronecan wins" (NOT DESIGN.md P4.0's inverted line)
- `src/drivers/dronecan/Kconfig`: `menuconfig DRIVERS_DRONECAN` `bool "dronecan"` `default n` `depends on PLATFORM_NUTTX` (+ sub-configs later, mirror `uavcan/Kconfig`).
- **Edit** `src/drivers/cyphal/Kconfig`: add `depends on !DRIVERS_DRONECAN` to `menuconfig DRIVERS_CYPHAL`.
- **Edit** `src/drivers/uavcan/Kconfig`: add `depends on !DRIVERS_DRONECAN` to `menuconfig DRIVERS_UAVCAN`.
- CMake `message(WARNING …)` belt-and-suspenders if both `CONFIG_` flags are set.

## Files to create in `src/drivers/dronecan/`
`CMakeLists.txt`, `Kconfig`, `module.yaml`, `dronecan.params.yaml`, `DronecanNode.hpp`, `DronecanNode.cpp` (minimal: `ScheduledWorkItem` on `wq:uavcan` + `extern "C" __EXPORT int dronecan_main(int, char**)` with start/stop/status, modeled on `cyphal/Cyphal.cpp:303` `print_usage` / `:309` `cyphal_main`).

## Build / enable (test target `ark_fmu-v6x`)
- Set `CONFIG_DRIVERS_DRONECAN=y`, `CONFIG_DRIVERS_UAVCAN=n` in `boards/ark/fmu-v6x/default.px4board` (or a scratch board config).
- `make ark_fmu-v6x`. **P4.0 acceptance:** empty module compiles + links behind `DRIVERS_DRONECAN`; generated headers present; `canard*` symbols don't collide (uavcan/cyphal off).
- Flash baseline: `/dronecan-flash-delta ark_fmu-v6x`.

## Then
P4.1 (port cyphal media `CanardInterface`/`CanardSocketCAN`/`CanardNuttXCDev` re-typed to v0 `CanardCANFrame`; implement `DronecanHandle` + `DronecanRxRouter` + `DroneCANCodec` + `FakeCanardInterface`; **codec round-trip + decode-inversion contract test pass before any bridge**) → P4.2 (3-type flash measurement gate). All per `DESIGN.md §8`.

## Decisions already locked (don't re-litigate)
Configure-time codegen (not build-time); Kconfig depends on cyphal+uavcan side (§2.1, "dronecan wins"); 4 submodules pinned to AP; `DRONECAN_CXX_WRAPPERS` off; pool flags TAO=1/CANFD=0/DEADLINE=1/SEM=0 (40 B block).
