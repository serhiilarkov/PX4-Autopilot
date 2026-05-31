# DroneCAN libcanard migration — execution plan & tracker

Living tracker for replacing PX4's libuavcan DroneCAN-v0 driver (`src/drivers/uavcan/`, ~124 KB) with a libcanard-based `src/drivers/dronecan/` (~53 KB target, **~64 KB flash reclaimed** on `ark_fmu-v6x`: 99.9% → ~96.6%).

- **Frozen briefing (required prior reading):** [`../DRONECAN_LIBCANARD_MIGRATION.md`](../DRONECAN_LIBCANARD_MIGRATION.md) — motivation, flash evidence, the three reference points, the architectural deltas, the open questions. Do not edit; it is the stable design input.
- **Flash evidence / prior findings:** project memory `project-flash-bloat-icf`.
- **Execution model — *tactical backbone*:** a human-driven, plan-mode-gated sequential build, with Claude Code workflows dropped in only for the fan-out / cross-check phases. Workflows never drive the safety-critical sequential build; they understand, extract, port-in-parallel, and review.

## Locked design decisions (§2.1 — do not re-litigate)

1. **Vendor the legacy libcanard** (`dronecan/libcanard`) + `dronecan_dsdlc` as submodules under the new driver. *Not* cyphal's modern (Cyphal-v1) libcanard.
2. **`dronecan` ⊥ `cyphal`, `dronecan` wins.** Kconfig `DRIVERS_CYPHAL depends on !DRIVERS_DRONECAN` + a belt-and-suspenders CMake `message(WARNING)`. This resolves the `canard*` symbol collision — the two libcanard generations never co-link.
3. **New module `dronecan`** (`src/drivers/dronecan/`, CLI `dronecan start`), mutually exclusive with the old `uavcan`; a board migration is one Kconfig flip (`DRIVERS_UAVCAN` off → `DRIVERS_DRONECAN` on).
4. **Params `UAVCAN_*` → `DC_*`** with an idempotent startup migration that imports legacy values (16-char name limit makes `DC_` roomier).
5. **Preserve DNA + SD-card firmware serving** (`BasicFileServer` + `FirmwareUpdateTrigger` + FW DB under `/fs/microsd/ufw`). In scope, not deferred.

## Phases → Claude Code primitive

| Phase | Work | Primitive / tool | Gate | Status |
|---|---|---|---|---|
| **P0** Scaffold | docs, memory, reusable tooling, lock the plan | interactive + memory | — | ✅ done |
| **P1** Understand | reference maps of AP_DroneCAN / cyphal / uavcan | workflow `dronecan-reference-map` → `REFERENCE_MAP.md` | — | ✅ done |
| **P2** Acceptance spec | 16 bridges + 3 actuators → field-map oracle | workflow `dronecan-acceptance-spec` → `ACCEPTANCE_SPEC.md` | — | ✅ done |
| **P3** Design + spike | architecture; resolve §8 (esp. Q1 codec layer); vendor + generate 2-3 codecs, measure `.text` | workflow N-angle judge panel → **plan-mode lock** | 🔒 design lock | ✅ locked |
| **P4** Vertical slice | GNSS + battery + ESC + DNA + SD-FW on `ark_fmu-v6x`, behind `DRIVERS_DRONECAN` | interactive backbone + `/dronecan-flash-delta` → PR | 🔒 slice proven | ⏳ P4.3 (P4.0–P4.2 ✅) |
| **P5** Remaining bridges | 12 sensors + aux controllers | workflow `dronecan-port-bridge` (worktree isolation) | — | ⬜ |
| **P6** Heavy subsystems | MAVLink param bridge, RemoteID, time sync, log fwd, param-migration fn | interactive backbone | — | ⬜ |
| **P7** Validate + cutover | size before/after, ESC latency, SITL/HIL; Kconfig/`rcS` switch | interactive + workflow `dronecan-review` | 🔒 cutover | ⬜ |

**Sign-off gates (yours):** after **P3** (design lock), after **P4** (slice proven on a hardware-sized build), before **P7** (cutover). Each gate = a plan-mode review you approve before the next phase starts.

## Reusable toolkit (committed under `.claude/`)

| Tool | Phase | What it does | Runnable |
|---|---|---|---|
| `workflows/dronecan-reference-map.js` | P1 | 8 parallel readers → consolidated `REFERENCE_MAP.md` | **now** |
| `workflows/dronecan-acceptance-spec.js` | P2 | per-bridge field-map extraction → `ACCEPTANCE_SPEC.md` | **now** |
| `workflows/dronecan-design.js` | P3 | 4 architects → 3-lens judge panel → synthesized `DESIGN.md` | **now** |
| `workflows/dronecan-port-bridge.js` | P5 | port N bridges in worktrees, verify each vs the spec | after P4 (needs the example bridge) |
| `workflows/dronecan-review.js` | P7 | adversarial review vs spec + flash-size gate | after P4 |
| `skills/dronecan-flash-delta/` | P4–P7 | build a board, report DroneCAN `.text`/`.rodata` delta (§9.1) | **now** |

Run a workflow by asking me, e.g. *"run the dronecan-reference-map workflow"*, or `/dronecan-reference-map` once saved. Skill: `/dronecan-flash-delta <board> [ref-before] [ref-after]`.

## Feature-parity checklist (P4/P5/P6 — the acceptance surface, §5)

### Sensor bridges (16)
- [ ] **GNSS** — `gnss.Fix`/`Fix2`/`Auxiliary` (+`ardupilot.gnss.RelPosHeading`) → `sensor_gps`  *(P4 slice)*
- [ ] GNSS relative — `ardupilot.gnss.RelPosHeading` → `sensor_gnss_relative`
- [ ] Baro — `air_data.StaticPressure`/`StaticTemperature`/`RawAirData` → `sensor_baro`
- [ ] Mag — `ahrs.MagneticFieldStrength`/`...2` → `sensor_mag`
- [ ] Accel — `ahrs.RawIMU` → `sensor_accel`
- [ ] Gyro — `ahrs.RawIMU` → `sensor_gyro`
- [ ] Airspeed — `air_data.IndicatedAirspeed`/`TrueAirspeed`/`StaticTemperature` → `airspeed`
- [ ] Diff pressure — `air_data.RawAirData` → `differential_pressure`
- [ ] **Battery** — `power.BatteryInfo` (+`BatteryInfoAux`, `cuav…CBAT`) → `battery_status`/`battery_info`  *(P4 slice)*
- [ ] Flow — `com.hex.equipment.flow.Measurement` → `sensor_optical_flow`
- [ ] Rangefinder — `range_sensor.Measurement` → `distance_sensor`
- [ ] Hygrometer — `dronecan.sensors.hygrometer.Hygrometer` → `sensor_hygrometer`
- [ ] Fuel tank — `ice.FuelTankStatus` → `fuel_tank_status`
- [ ] ICE status — `ice.reciprocating.Status` → `internal_combustion_engine_status`
- [ ] Safety button — `ardupilot.indication.Button` → `button_event`
- [ ] Battery filtered mode (`DC_SUB_BAT==2` → internal battery lib)

### Actuators
- [ ] **ESC** — out `esc.RawCommand` ≤400 Hz; in `esc.Status`/`StatusExtended` → `esc_status`  *(P4 slice)*
- [ ] Servo — `actuator.ArrayCommand` @50 Hz
- [ ] Hardpoint — `hardpoint.Command`

### Services & node management
- [ ] **DNA-v0 centralized allocation server**  *(P4 slice)*
- [ ] **SD-card FW/file server** (`BasicFileServer` + `FirmwareUpdateTrigger` + FW DB `/fs/microsd/ufw`)  *(P4 slice, decision 5)*
- [ ] NodeInfoRetriever / NodeStatusMonitor → `dronecan_node_status`
- [ ] NodeInfoPublisher → `device_information`; `can_interface_status`
- [ ] GlobalTimeSync master/slave
- [ ] Log forwarding (`debug.LogMessage` → `px4_log`)
- [ ] MAVLink remote-param bridge (GetSet / ExecuteOpcode / RestartNode)  *(P6)*

### Aux controllers (`CONFIG_UAVCAN_*`-gated)
- [ ] ArmingStatus, Beep (`tune_control`→`BeepCommand`), RGB LED (`LightsCommand`), SafetyState, RemoteID (bidirectional Open Drone ID), GNSS RTCM/MovingBaseline + PPK `gps_dump`

### Params + boot
- [ ] `DC_ENABLE` / `DC_NODE_ID` / `DC_BITRATE` / `DC_SUB_*` / `DC_PUB_*` / `DC_EC_*` / `DC_SV_*` + the `UAVCAN_*`→`DC_*` migration fn
- [ ] `rcS`: `if param greater DC_ENABLE 0; then dronecan start`

## Open questions (resolve in P3, §8)

> P1 produced **draft recommendations** for Q1/Q3/Q5/Q7 in `REFERENCE_MAP.md` §8 — to confirm (and, for Q1, measure 2-3 types) at the P3 design-lock gate.

1. **Typed codec layer** — ArduPilot's header-only `Canard::Publisher/Subscriber` + `cxx_iface` trait, or a lighter PX4 shim? *Measure 2-3 types early* — **highest-leverage**; the whole point is to avoid libuavcan's per-type explosion.
2. Scope/phasing — minimal vertical slice first ✓ (baked into P4).
3. DNA-server source — port AP's `StorageManager`-backed server vs re-host the current `UavcanServers` on libcanard. Either must keep allocation **and** SD-card FW serving.
4. MAVLink param bridge & RemoteID — sequence after the core slice ✓ (P6).
5. Memory model — legacy internal-pool sizing vs today's `HeapBasedPoolAllocator` (soft/hard blocks); provide a `dronecan shrink`?
6. Param-migration details — exact name map (16-char limit), run timing/idempotency, legacy retirement, how `rcS` selects `dronecan` vs `uavcan` during transition.
7. Validation — ESC output latency (immediate-flush vs WQ), SITL/HIL DroneCAN, `size -A` before/after on constrained boards.

## Current status

**2026-05-30:** P0 scaffolding complete. **P1 done** — `REFERENCE_MAP.md` written (8 readers + synthesis, ~760K tok) and spot-checked (PX4-side citations accurate); §7 caught real correctness traps, §8 carries draft recommendations for Q1/Q3/Q5/Q7. **P2 done** — `ACCEPTANCE_SPEC.md` (982 lines, 18 bridges, 584 citations), spot-checked accurate; a clean per-field porting oracle. Spec self-flags 3 spots for extra human review when ported: Battery (4 mode-paths; some NAN rows originate in `src/lib/battery`), GNSS Fix2 covariance fallthrough (`gnss.cpp:217`, latent bug), ESC `failures` (derived from uORB inputs, not the Status msg). **Design LOCKED (plan approved 2026-05-30); P4 in progress** on branch `dronecan-libcanard`. **P4.0:** 4 DroneCAN submodules vendored + pinned to AP commits (libcanard `601ed35`, dronecan_dsdlc `de47a89`, DSDL `04e0e81`, pydronecan `08cda37`); **codegen validated standalone** in PX4's env (206 headers / 176 srcs, rc=0) — confirms Q1: `_ID`/`_SIGNATURE`/`_MAX_SIZE` `#define`s present, `cxx_iface` trait `#ifdef DRONECAN_CXX_WRAPPERS`-guarded (off by default), decode returns TRUE-on-failure. **Implementation notes / deviations:** codegen wired **configure-time** (cyphal idiom — compiled `.c` needs it; DESIGN.md's "build-time" note was for the header-only libuavcan case); Kconfig mutual-exclusion follows **§2.1** ("dronecan wins" → `depends on !DRIVERS_DRONECAN` on cyphal + uavcan), not DESIGN.md P4.0's inverted paraphrase.

**2026-05-30 — P4.0 scaffold ✅ (build green on `ark_fmu-v6x`).** Created `src/drivers/dronecan/`: `CMakeLists.txt` (4× `px4_add_git_submodule`, configure-time `dronecan_dsdlc` codegen with `message(FATAL_ERROR)` on rc≠0, `file(GLOB)` of generated `src/*.c`, canard flags `TAO=1/CANFD=0/DEADLINE=1/SEM=0`, `DRONECAN_CXX_WRAPPERS` left undefined, `remove_definitions(-Werror)`, FATAL belt-and-suspenders if `CONFIG_DRIVERS_UAVCAN`/`CONFIG_DRIVERS_CYPHAL` co-set); `Kconfig` (`menuconfig DRIVERS_DRONECAN`, `depends on PLATFORM_NUTTX`); §2.1 edits to `cyphal/Kconfig` + `uavcan/Kconfig` (`depends on !DRIVERS_DRONECAN`); minimal `module.yaml`; `dronecan.params.yaml` (`DC_ENABLE`/`DC_NODE_ID`/`DC_BITRATE`); minimal `DronecanNode` (`ScheduledWorkItem` on `wq:uavcan` + `dronecan {start|status|stop}` CLI). Board toggle: `boards/ark/fmu-v6x/default.px4board` → `CONFIG_DRIVERS_DRONECAN=y`, `DRIVERS_UAVCAN` removed (uncommitted; the toggle under test). **Result:** `make ark_fmu-v6x` exit 0; 178 module objects (canard.c + 176 generated + node); `libdrivers__dronecan.a` linked; `dronecan_main` + `canardInit` are `T` symbols in the ELF; **0 uavcan objects** (mutual exclusion confirmed); whole-module compile **clean** (the build's lone `warning:` is the pre-existing board `ARCH_CHIP_UNSET`). FLASH 92.46% (1,817,752 B) — empty-scaffold baseline; the meaningful number is the P4.2 3-type seam measurement.

**2026-05-30 — P4.1 transport + codec + dispatch ✅ (contract test passes; compiles on `ark_fmu-v6x`).** Added the platform-free transport stack to `src/drivers/dronecan/`: `CanardInterface` (v0 media base + `DronecanRxFrame`, re-typed to `CanardCANFrame`); `DroneCANCodec` (the two error boundaries — `decodeTransfer` is the single TRUE-on-failure negation; `encodePayload`/`encodeBroadcast` the single 0-length-failure check; `TypeDescriptor` POD as data); `DronecanRxRouter` + `RxSubscriberBase` (per-instance `[dtid % DC_RX_BUCKETS]` buckets, broadcast→all / service→first, `accept`+`dispatch` walk the same bucket so the accepted signature always matches the decoder); `DronecanHandle` (the sole canard boundary — `canardInit` over a per-instance 8 KB static pool, the two `user_reference`→`this` trampolines, `receive`/`transmit`/`broadcast`/`requestOrRespond`/`cleanupStale`/`poolStats`, constructor-injected media, pool-exhaustion exposed as an observable counter). **Deliberately platform-free** (canard + std only, no perf/hrt/log) so the whole stack is drivable off-target; the PX4 glue (perf, hrt, real CAN media, logging) lands in the node in P4.3. **Gate met:** `test/dronecan_codec_test.cpp` (one-command `test/run_codec_test.sh`) passes against the real `canard.c` + generated `NodeStatus` codec — decode inversion (stub + real wrong-length, too-short and too-long), encode→decode identity, and full fake-frame dispatch (encode→broadcast→TX-drain→`FakeCanardInterface` loopback→`canardHandleRxFrame` reassembly→trampolines→router→subscriber→decode→identity; transfer-id auto-increment verified). All transport sources also compile clean on `ark_fmu-v6x` (gc-stripped until the node wires them in P4.3); build stays at 1 pre-existing warning. **Deviations:** the gate runs as a standalone host program, not `px4_add_unit_gtest` — dronecan is NuttX-only so its CMakeLists isn't processed on the SITL test build, which would leave a host gtest dead; CI integration (SITL-buildable codec, or relaxing the Kconfig gate) is a follow-up. `flush()` (ESC-only TX filter; needs queue-walking past non-ESC frames) and the real `CanardNuttXCDev`/`CanardSocketCAN` backends are deferred to P4.7/P4.3 (not needed for the codec gate). **Next:** P4.2 — reference 3 type descriptors (`gnss.Fix2`, `power.BatteryInfo`, `esc.RawCommand`) and run `/dronecan-flash-delta ark_fmu-v6x` to confirm per-type cost is flat (generated `_encode`/`_decode` only, no template/vtable growth).

**2026-05-30 — P4.2 Q1 seam measurement ✅ GO (codec layer is flat).** Measured per-type vs fixed cost directly from the `ark_fmu-v6x` build objects (`arm-none-eabi-size`/`nm`) — isolating the seam more precisely than a gc-stripped full-binary delta. **Per-type cost is ONLY the generated marshalling:** `gnss.Fix2` 1342 B, `power.BatteryInfo` 788 B, `esc.RawCommand` 220 B of `.text`, each with 0 data / 0 bss (no per-type static tables). **Fixed transport cost is O(1)** regardless of type count: `DroneCANCodec` 92 + `DronecanRxRouter` 116 + `DronecanHandle` 440 = **648 B** `.text` total. `nm --defined-only -C` on the three transport objects shows ONLY type-erased symbols (`encodePayload`/`decodeTransfer`/`encodeBroadcast` over `void*`+fn-ptr+`TypeDescriptor`; router/handle over `TypeDescriptor`) — **zero** per-type symbols, no `Publisher<T>`/`Subscriber<T>`/vtable/`msg_buf[MAX_SIZE]` instantiation (the libuavcan per-type explosion that is the ~64 KB bloat source). **Verdict: GO** — each new bridge type adds only its irreducible generated `_encode`/`_decode`; the typed surface stays flat in message count, validating the Q1 thesis the whole reclaim rides on. The optional `decode<T>` template was dropped for the non-templated `decodeTransfer` + thin per-type wrappers (DESIGN §9.2 "already ergonomic enough"), so there is no template to fold. **Note:** the *integrated* `/dronecan-flash-delta` is uninformative while the transport is gc-stripped (unreferenced); the end-to-end ~64 KB reclaim is the **P4.9** gate, measured once the slice references the types. **Next:** P4.3 — `DronecanNode` lifecycle: wire the handle + real CAN media on `wq:uavcan`, lazy first-`Run()` init, `transmit/receive/transmit` under `_node_mutex`, `canardSetLocalNodeID(DC_NODE_ID)`, NodeStatus 1 Hz, CLI, `migrateLegacyParams()`, rcS gate. **→ Resume guide: [`RESUME_P4.md`](RESUME_P4.md)**.
