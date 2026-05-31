# Replacing PX4's libuavcan DroneCAN driver with a libcanard implementation — Design Briefing

**Status:** investigation & measurement complete; **key design decisions locked (§2.1)**; detailed design & implementation plan not yet started. **Audience:** the next session, which will (1) clean-room *design* a libcanard-based DroneCAN-v0 driver for PX4, produce an implementation plan, and (2) implement it. **This document is required prior reading.** It does not contain the design — it gives you the motivation, the hard evidence, the three reference points you will engineer from, the architectural differences you must respect, and the open questions you must decide.

All file references are `path:line`. PX4 root = `/home/jake/code/jake/PX4-Autopilot`. ArduPilot root = `/home/jake/code/jake/ardupilot`.

---

## 1. Motivation

### 1.1 libuavcan is bloated, and the bloat is irreducible *within libuavcan*

PX4's DroneCAN-v0 stack is the C++ template library `libuavcan` (`src/drivers/uavcan/libdronecan/libuavcan/`, a fork of the abandoned UAVCAN-v0 `libuavcan`). It is large and, as measured exhaustively, cannot be meaningfully shrunk by any folding/de-templatization technique.

Measured on `ark_fmu-v6x_default` (gcc-arm-none-eabi 10.2.1, `-Os`):

| "uavcan" `.text` | size |
|---|--:|
| libuavcan **library** (`uavcan::` — templates + transport/node/protocol core) | **94.3 KB** |
| PX4 **bridge** glue (`UavcanNode`, sensor bridges, controllers) | 26.3 KB |
| STM32 CAN driver (`uavcan_stm32h7::`) | 3.2 KB |
| **total DroneCAN subsystem** | **~124 KB** |

Why it can't be folded:
- **Identical Code Folding (ICF), the gold-standard redundancy detector, folds only 3,194 B of the entire ~124 KB** (relinked the same objects with `ld.lld --icf=all`; 46 of 237 image-wide folds were uavcan). The library is ~96% *genuinely-distinct* machine code.
- **Source de-templatization yields ≤ 0.** Two measured experiments: hoisting `registerDataType` plumbing into a non-template base *regressed* `.text` by +3,184 B (broke inlining); precomputing the DSDL signature CRC at codegen saved **0 B** (GCC already constant-folds it at `-Os`).
- The reason is structural: every DroneCAN message type instantiates its own `GenericSubscriber<T>`, `GenericPublisher<T>`, `ServiceClient<T>`, `MethodBinder<...>`, `TransferListener`, plus a vtable. That per-type machinery — *not* the marshalling (only ~5 KB of `encode`/`decode` total) — is the 94 KB. Each instantiation is genuinely different code, so nothing folds it. The library author already factored all type-*independent* code into non-template bases (`GenericSubscriberBase`, `GenericPublisherBase`, the `uc_*.cpp` transport/marshal cores), so there is no remaining source win.

**Conclusion: the only way to reduce this subsystem is a different architecture.**

### 1.2 libuavcan is hard to work with

Beyond size: it is C++03 template metaprogramming with a dead upstream, intricate transfer/marshalling templates, and a per-type instantiation model that makes adding a message type or reasoning about code size opaque. Maintainers have already anticipated replacing it (the closed LTO PR #26462 thread, julianoes: *"closing in favor of … future libuavcan replacement"*).

### 1.3 The evidence: ArduPilot's libcanard implementation is ~3× smaller

ArduPilot implements the *same DroneCAN protocol* with **libcanard** — a small C library that dispatches all message types through one runtime code path (keyed by data-type-ID at runtime) instead of per-type C++ templates. Built ArduCopter for `ARKV6X` with the **same gcc-10 toolchain**:

| DroneCAN component | PX4 (libuavcan, C++ templates) | ArduPilot (libcanard, C) |
|---|--:|--:|
| Protocol **library/core** | 94.3 KB (`uavcan::`) | **2.6 KB** (libcanard core) |
| Generated **per-type codecs** | *(within the 94.3)* | 27.7 KB |
| → **protocol implementation total** | **94.3 KB** | **30.3 KB** |
| Driver / bridge layer | 26.3 KB | 22.9 KB (`AP_DroneCAN`) |
| **DroneCAN subsystem total** | **~124 KB** | **~53 KB** |

**The protocol implementation is ~64 KB / ~3× smaller in libcanard.** The driver/bridge layers are comparable (26 vs 23 KB) — that's application-facing translation code either way. The entire delta lives in the *library*: libcanard's transport core is 2.6 KB for *all* message types; libuavcan instantiates per-type machinery for each. For context, whole-firmware: ArduCopter ARKV6X = 1.58 MB / 383 KB free; PX4 ark_fmu-v6x = 1.96 MB / **99.9% full**. Reclaiming ~64 KB would take a v6x from 99.9% to ~96.6% — real headroom on the most flash-constrained boards.

---

## 2. Goal & strategy

**Goal:** replace PX4's libuavcan-based DroneCAN-v0 driver with a libcanard-based one, preserving all current functionality, and recover the flash.

**Strategy — clean-room engineering from ArduPilot, expressed in PX4 idioms.** The new driver is a *fusion of three in-repo reference points*, each of which you must study:

1. **DroneCAN-v0 protocol semantics & libcanard usage ← ArduPilot `AP_DroneCAN`** (§3). This is the *behavioral* reference: how the legacy libcanard API is driven, how DSDL-v0 messages map to/from domain data, the DNA-v0 protocol, the pub/sub/service patterns. You study it and re-express it; you do not copy ArduPilot-specific plumbing.
2. **PX4 integration patterns ← the in-tree `cyphal` driver** (§4). PX4 *already runs libcanard* (for Cyphal/UAVCAN v1). `src/drivers/cyphal/` is the canonical example of libcanard living inside PX4: the work-queue scheduling model, the CAN media abstraction, the **uORB bridge pattern**, and param-driven configuration. Reuse these shapes directly.
3. **The acceptance spec ← the current libuavcan `UavcanNode`** (§5). The exhaustive list of sensors, actuators, services, uORB topics, params, and CLI commands the replacement must reproduce.

The synthesis: **PX4 plumbing (from cyphal) + DroneCAN-v0 semantics (from ArduPilot) + feature parity (from current `UavcanNode`).**

### 2.1 Locked design decisions (constraints — do not re-litigate)

These are settled by the project owner. The design session operates within them.

1. **Vendor the legacy libcanard.** Import the *same* legacy DroneCAN-v0 libcanard ArduPilot uses (`dronecan/libcanard`) plus `dronecan_dsdlc`, as submodules under the new driver. (Not the modern Cyphal libcanard that `cyphal` uses.)
2. **`dronecan` and `cyphal` are mutually exclusive; `dronecan` wins.** Enforced in Kconfig: `DRIVERS_CYPHAL` gains `depends on !DRIVERS_DRONECAN`, so selecting the new driver forces `cyphal` off. If a board config sets both `CONFIG_` flags, Kconfig `olddefconfig` drops cyphal with an unmet-dependency **warning** (dronecan takes precedence); add a CMake guard that also emits `message(WARNING …)` as belt-and-suspenders. This *resolves* the `canard*` symbol-collision risk — the two libcanard generations can never link into one binary.
3. **New module name `dronecan`; migration is a single Kconfig switch.** The new driver lives at `src/drivers/dronecan/` (CLI `dronecan start`). It is mutually exclusive with the old `uavcan` (libuavcan) driver — two implementations of the same protocol — so moving a board old→new is one Kconfig flip (`DRIVERS_UAVCAN` off → `DRIVERS_DRONECAN` on).
4. **Parameters rename `UAVCAN_*` → `DC_*`, with automatic migration.** The new module ships `DC_*` params (same semantics as today's set, §5) plus a **startup migration function** (in the `dronecan` module) that detects legacy `UAVCAN_*` values in param storage and copies/converts them into the corresponding `DC_*` params (idempotent — runs once, no-op thereafter). The 16-char param-name limit makes `DC_` strictly roomier than `UAVCAN_`.
5. **Preserve DNA + SD-card firmware serving.** Full parity on Dynamic Node-ID Allocation **and** serving firmware updates from the flight controller's SD card to connected nodes (today's `UavcanServers`: `BasicFileServer` + `FirmwareUpdateTrigger` + FW DB under `/fs/microsd/ufw`). In scope — not deferred.

---

## 3. Reference architecture: ArduPilot `AP_DroneCAN` (the behavioral reference)

Source: `libraries/AP_DroneCAN/` + `modules/DroneCAN/` in the ArduPilot tree.

- **Structure.** `AP_DroneCAN` (`AP_DroneCAN.h:81`, ~2000-line `.cpp`) — one instance per CAN bus; owns the `CanardInterface`, the DNA server, a thread, and *all* DSDL↔domain translation. `CanardInterface` (`AP_Canard_iface.h:10`, `.cpp`) wraps the libcanard C instance and bridges to `AP_HAL::CANIface`. `AP_DroneCAN_DNA_Server` (`AP_DroneCAN_DNA_Server.h:17`) is the dynamic-node-ID allocation server + node health/duplicate monitor. Deliberate design choice (`AP_DroneCAN.cpp:90-93`): DSDL↔struct translation is concentrated in `AP_DroneCAN` and the per-subsystem backends, never inside libcanard.

- **libcanard API style — LEGACY.** `canardInit(&canard, pool, size, onTransferReception, shouldAcceptTransfer, this)` registers two callbacks at init (`AP_Canard_iface.cpp:65`). TX = `canardBroadcastObj` / `canardRequestOrRespondObj` (`:99,136,168`); RX = `canardHandleRxFrame` (`:370`) which **calls back** into `should_accept` / `on_reception`. A single fixed memory pool inside the `CanardInstance` (`mem_pool`, sized by the `POOL` param) serves both RX reassembly states and TX queue items. This is **not** the modern `canardRxAccept`/`canardTxPush`/O1Heap API.

- **Typed pub/sub layer.** Header-only `Canard::` templates in `modules/DroneCAN/libcanard/canard/`: `Publisher<T>` (`publisher.h:82`), `Subscriber<T>` (`subscriber.h:35`), `Client`/`Server`. Dispatch uses a static `HandlerList::head[CANARD_NUM_HANDLERS][8]` indexed by `[driver_index][msgid % 8]` (`handler_list.h:147`); subscribers `link()` themselves at construction. The generated DSDL provides a `cxx_iface` trait (`static constexpr ID/SIGNATURE/MAX_SIZE` + `encode`/`decode` fn-ptrs) that the templates consume — this is what keeps the template layer thin. Registration example (`AP_DroneCAN.h:342`): `Canard::Subscriber<uavcan_equipment_esc_Status> esc_status_listener{esc_status_cb, _driver_index};`. Send (`AP_DroneCAN.cpp:854`): `esc_raw.broadcast(esc_msg)`.

- **Data flow (study these traces).** Incoming `gnss.Fix2` → libcanard reassembly → `Subscriber::handle_message` decodes → `AP_GPS_DroneCAN::handle_fix2_msg_trampoline` (`AP_GPS_DroneCAN.cpp:597`) resolves the backend by source node-ID and fills an `AP_GPS::GPS_State`. Battery (`BatteryInfo`) → `AP_BattMonitor_DroneCAN`. ESC status is handled *inside* `AP_DroneCAN::handle_ESC_status` (`AP_DroneCAN.cpp:1470`). Outgoing ESC: `SRV_send_esc()` packs `esc.RawCommand` and **immediately flushes TX** (`processTx(true)`, `:860`) for low latency; servos are rate-limited in the thread loop.

- **Threading.** **One dedicated thread per CAN bus** (`thread_create(... PRIORITY_CAN ...)`, `AP_DroneCAN.cpp:500`). Loop (`:511`): `delay_microseconds(100)` throttle → `canard_iface.process(1)` (1 ms of RX/TX pumping that blocks on a CAN-ISR binary semaphore) → periodic housekeeping (node status @1 Hz, DNA `verify_nodes` @5 s, servo send, params). **Note: PX4 will not use this thread model — see §6.2.**

- **DNA server.** Centralized two-step UID allocation (`handle_allocation`, `AP_DroneCAN_DNA_Server.cpp:506`) + node monitoring via periodic `GetNodeInfo`. Persists a node-ID↔unique-ID DB in `StorageManager`; mostly `Bitmask<128>` bookkeeping (~590 lines). Behaviorally equivalent to what PX4's `UavcanServers` does today.

- **DSDL codegen.** `modules/DroneCAN/dronecan_dsdlc/` (EmPy templates, driven by `pydronecan`) generates one C `.h`+`.c` per message: a plain `struct`, `<type>_encode(msg, buf[, tao])`, `<type>_decode(const CanardRxTransfer*, msg)` (**note: `decode` returns `true` on FAILURE**), `#define`s for `_ID`/`_SIGNATURE`/`_MAX_SIZE`, and (under `DRONECAN_CXX_WRAPPERS`) the `cxx_iface` trait. Output lands at `build/<BOARD>/modules/DroneCAN/libcanard/dsdlc_generated/`. **This generator is the natural source of PX4's v0 C codecs** (it emits legacy-libcanard-compatible code).

---

## 4. In-tree integration template: the PX4 `cyphal` driver (the PX4-idiom reference)

Source: `src/drivers/cyphal/`. This is libcanard already integrated into PX4 — copy these shapes.

- **Structure.** `CyphalNode : public ModuleParams, public px4::ScheduledWorkItem` (`Cyphal.hpp:108`) owns one `CanardHandle` (`CanardHandle.hpp:40`, wraps the libcanard instance + TX queue + allocator). `CanardInterface` (`CanardInterface.hpp:46`) is the abstract CAN-media layer with concrete `CanardSocketCAN` / `CanardNuttXCDev` backends (selected by `CONFIG_NET_CAN` vs `CONFIG_CAN`). Publishers/Subscribers/Managers reference the handle, never libcanard directly.

- **libcanard API style — MODERN (Cyphal v1).** `cyphal/libcanard` is OpenCyphal libcanard v3 with O1Heap: `canardInit(&alloc,&free)`, `canardTxPush`/`canardTxPeek`/`canardTxPop`, `canardRxAccept` (**poll/pull**, not callbacks), `canardRxSubscribe` (`CanardHandle.cpp:72,182,157,122,191`). RX payloads and TX frames are heap-allocated from an 8 KB O1Heap arena and must be freed by the app. **This differs fundamentally from the v0 legacy API** (§6.6) — the v0 port reshapes the *inside* of `CanardHandle`, but everything around it (media layer, scheduling, uORB bridges) is reusable.

- **uORB bridge — the reusable crown jewel.** Subscriber: after `canardRxAccept` returns a transfer, the matching subscription's `user_reference` is cast to `UavcanBaseSubscriber*` and its `callback()` runs `<type>_deserialize_` then `uORB::PublicationMulti<T>::publish()` (canonical example `Subscribers/udral/Battery.hpp:99-122`; raw passthrough variant `Subscribers/uORB/uorb_subscriber.hpp:76`). Publisher: `uORB::Subscription::update()` → `<type>_serialize_` → `CanardHandle::TxPush(...)` (`Publishers/uORB/uorb_publisher.hpp:63`, DSDL example `Publishers/udral/Gnss.hpp:61`). **This decode→`orb publish` / `orb subscribe`→encode pattern is version-independent and is exactly what the v0 bridges should mirror.**

- **Scheduling — work queue, not a thread.** `ScheduledWorkItem` on `wq:uavcan`, `ScheduleOnInterval(3 ms)` (~333 Hz) *complemented by CAN/uORB events* (`Cyphal.cpp:69,141`). `Run()` (`Cyphal.cpp:156`): lock `_node_mutex`, lazy init, on `parameter_update` reconfigure pub/sub, send periodic node messages, then **`transmit(); receive(); transmit();`** to drain TX, process all RX, flush responses. A *second* WorkItem `UavcanMixingInterface` (`OutputModuleInterface`) runs ESC output on the same WQ, scheduled by `MixingOutput`.

- **Param-driven dynamic pub/sub.** Each pub/sub maps to an `int32` PX4 param (a Cyphal port-ID; -1 = disabled). `PublicationManager`/`SubscriptionManager` lazily instantiate a pub/sub the first time its param becomes valid, from a compile-time table of factory lambdas gated by `CONFIG_CYPHAL_*` (`PublicationManager.hpp:116`, `SubscriptionManager.hpp:130`). `UavcanParamManager` bridges PX4 params ↔ Cyphal registers. (v0 uses fixed data-type IDs, so this port-ID indirection can be *simplified* — see §6.)

- **DSDL codegen.** Nunavut `nnvg` at CMake-configure time → `<type>_serialize_`/`_deserialize_` (`CMakeLists.txt:42-50`). The v0 driver will instead wire ArduPilot's `dronecan_dsdlc` (§3) into an equivalent CMake step.

- **Startup.** `CONFIG_DRIVERS_CYPHAL=y` per board; `cyphal start` gated by `CYPHAL_ENABLE`. Note `ROMFS/.../rcS:540-548` already makes UAVCAN-v0 and Cyphal **mutually exclusive at boot** (UAVCAN_ENABLE takes precedence) — relevant to §6.6.

---

## 5. Acceptance spec: what the current libuavcan `UavcanNode` does (must be preserved)

Source: `src/drivers/uavcan/` (the PX4 layer above the library). This is the feature inventory the replacement must reproduce.

- **Lifecycle.** `UavcanNode : px4::ScheduledWorkItem, ModuleParams` (`uavcan_main.hpp:181`), singleton, on `wq:uavcan`. `ScheduleOnInterval(3 ms)` **plus** CAN-RX-IRQ `ScheduleNow()` wakeup (`uavcan_main.cpp:485-497`). CLI: `uavcan {start|status|stop|shrink|update|param {set|get|list|save} <node>|reset <node>}`. Memory: `HeapBasedPoolAllocator`, block size 48, soft 250 / hard 500 blocks.

- **16 incoming sensor bridges** (`src/drivers/uavcan/sensors/`), each with per-sensor compile gate (`CONFIG_UAVCAN_SENSOR_*`) **and** runtime gate (`UAVCAN_SUB_*`), redundancy channels (multi-instance uORB per source node), device-ID encoding, and NodeInfoPublisher capability registration:

| Bridge | DroneCAN message(s) → | uORB topic |
|---|---|---|
| GNSS | `gnss.Fix`/`Fix2`/`Auxiliary` (+`ardupilot.gnss.RelPosHeading`) | `sensor_gps` |
| GNSS relative | `ardupilot.gnss.RelPosHeading` | `sensor_gnss_relative` |
| Baro | `air_data.StaticPressure`/`StaticTemperature`/`RawAirData` | `sensor_baro` |
| Mag | `ahrs.MagneticFieldStrength`/`...2` | `sensor_mag` |
| Accel | `ahrs.RawIMU` | `sensor_accel` |
| Gyro | `ahrs.RawIMU` | `sensor_gyro` |
| Airspeed | `air_data.IndicatedAirspeed`/`TrueAirspeed`/`StaticTemperature` | `airspeed` |
| Diff pressure | `air_data.RawAirData` | `differential_pressure` |
| Battery | `power.BatteryInfo` (+`BatteryInfoAux`, `cuav…CBAT`) | `battery_status`/`battery_info` |
| Flow | `com.hex.equipment.flow.Measurement` | `sensor_optical_flow` |
| Rangefinder | `range_sensor.Measurement` | `distance_sensor` |
| Hygrometer | `dronecan.sensors.hygrometer.Hygrometer` | `sensor_hygrometer` |
| Fuel tank | `ice.FuelTankStatus` | `fuel_tank_status` |
| ICE status | `ice.reciprocating.Status` | `internal_combustion_engine_status` |
| Safety button | `ardupilot.indication.Button` | `button_event` |
| (battery filtered mode `UAVCAN_SUB_BAT==2` runs the internal battery lib) | | |

- **Actuator output** via two independent `MixingOutput`/`OutputModuleInterface` WorkItems: `UavcanEscController` (out: `esc.RawCommand` @≤400 Hz; in: `esc.Status`/`StatusExtended` → `esc_status` + failure parsing), `UavcanServoController` (`actuator.ArrayCommand` @50 Hz), `UavcanHardpointController` (`hardpoint.Command`). Output scaling via `UAVCAN_EC_*` / `UAVCAN_SV_*` params.

- **Services & node management:** centralized DNA server + FW/file server + FW DB management (`UavcanServers`, gated `UAVCAN_ENABLE>1`); `NodeInfoRetriever`; `NodeStatusMonitor` → `dronecan_node_status`; `NodeInfoPublisher` → `device_information`; `can_interface_status`; GlobalTimeSync master/slave; log forwarding (`debug.LogMessage` → `px4_log`); **MAVLink remote-param bridge** (GetSet/ExecuteOpcode/RestartNode state machine in `Run()`, `uavcan_main.cpp:771-970`).

- **Aux controllers** (each `CONFIG_UAVCAN_*`-gated): ArmingStatus, Beep (`tune_control`→`BeepCommand`), RGB LED (`LightsCommand`), SafetyState, RemoteID (bidirectional Open Drone ID), GNSS RTCM/MovingBaseline injection + PPK `gps_dump`.

- **uORB surface:** publishes ~20 topics (sensor topics + `esc_status`, `dronecan_node_status`, `device_information`, `can_interface_status`, `uavcan_parameter_value`, `vehicle_command_ack`, `gps_dump`, `open_drone_id_arm_status`); subscribes `actuator_motors`/`actuator_outputs`, `actuator_armed`, `tune_control`, `vehicle_command`, `parameter_update`, `uavcan_parameter_request`, `gps_inject_data`, and the RemoteID input set.

- **Params:** `UAVCAN_ENABLE` (0=off/1=sensors/2=+DNA+FW/3=+outputs), `UAVCAN_NODE_ID`, `UAVCAN_BITRATE`, `UAVCAN_ESC_IFACE`, the `UAVCAN_SUB_*` per-sensor enables, `UAVCAN_PUB_*`, light/range/fuel params, plus `UAVCAN_EC_*`/`UAVCAN_SV_*` mixer params. Boot: `rcS:541` `if param greater UAVCAN_ENABLE 0; then uavcan start`.

Key files: `uavcan_main.{cpp,hpp}`, `uavcan_servers.{cpp,hpp}`, `node_info.{cpp,hpp}`, `sensors/sensor_bridge.{cpp,hpp}` + 15 sensor `.cpp`, `actuators/{esc,servo,hardpoint}.cpp`, `{arming_status,beep,rgbled,safety_state,remoteid,logmessage}.*`, `uavcan_params.yaml`, `module.yaml`, `Kconfig`, `CMakeLists.txt`.

---

## 6. Architectural differences PX4 ⟷ ArduPilot, and the design implications

You are translating ArduPilot's *behavior* into PX4's *architecture*. The differences below are the crux; get them right and the rest follows.

| Dimension | ArduPilot | PX4 | Implication for the new driver |
|---|---|---|---|
| **6.1 Data model** | Singleton sensor backends (`AP_GPS_DroneCAN` fills an `interim_state` that a frontend polls) | **uORB** pub/sub topics, multi-instance | The bridge *is* the integration point: decode → `uORB::PublicationMulti::publish()`. No frontend/backend split. **Simpler than ArduPilot.** Reuse the cyphal subscriber pattern; the message→uORB field mappings already exist in the current libuavcan bridges (§5) — port those mappings, not ArduPilot's. |
| **6.2 Scheduling** | One dedicated **thread** per bus, blocks on CAN-ISR semaphore | **`ScheduledWorkItem`** on `wq:uavcan`, fixed tick + CAN-RX-IRQ `ScheduleNow()` | Use the **PX4 work-queue model** (both cyphal and current `UavcanNode` already do exactly this). Do **not** port ArduPilot's thread. Keep the `transmit(); receive(); transmit();` drain (cyphal) and the IRQ-wakeup trampoline (current uavcan, `uavcan_main.cpp:485`). |
| **6.3 Parameters** | `AP_Param` `GroupInfo` (`CAN_Dx_UC_*`) | PX4 param system (yaml + `DEFINE_PARAMETERS`) | **Rename `UAVCAN_*` → `DC_*`** (decision 4) and ship a startup migration that imports legacy `UAVCAN_*` values. Preserve the *semantics* of the current set (§5): `DC_ENABLE`, `DC_NODE_ID`, `DC_BITRATE`, `DC_SUB_*`, `DC_PUB_*`, `DC_EC_*`/`DC_SV_*`, etc. v0 uses fixed data-type IDs, so pub/sub config collapses to compile + `DC_SUB_*` gating (like today's `make_all()`), not cyphal's dynamic port-ID layer. |
| **6.4 CAN media** | `AP_HAL::CANIface` | `CanardInterface` + `CanardSocketCAN`/`CanardNuttXCDev` | **Reuse cyphal's media layer as-is** — it is protocol-version-agnostic (raw CAN frames). |
| **6.5 Outputs / mixing** | `SRV_Channels` | `MixingOutput` / `OutputModuleInterface` | Use the **current `UavcanNode` ESC/servo path** (two WorkItems, `UAVCAN_EC_*`/`UAVCAN_SV_*` scaling). |
| **6.6 libcanard version** | **Legacy** v0 libcanard (callbacks, internal pool, `canardBroadcast`/`canardHandleRxFrame`) | cyphal uses **modern** v1 libcanard (poll, O1Heap, `canardTxPush`/`canardRxAccept`) | DroneCAN v0 requires the *legacy* libcanard (incompatible wire protocol/addressing vs Cyphal v1); PX4 vendors it (decision 1). Reuse the cyphal `CanardHandle` wrapper *shape*, but rewrite its interior to the legacy API (model on ArduPilot's `CanardInterface`: `canardInit` + the two callbacks, `canardBroadcastObj`, callback-driven `canardHandleRxFrame`, single internal pool). The `canard*` symbol collision between the two libcanard generations is **resolved by the mutual exclusion in decision 2** — `dronecan` and `cyphal` never link into the same binary, so the duplicate symbols never meet. |
| **6.7 DSDL codegen** | `dronecan_dsdlc` (C, EmPy) → `_encode`/`_decode` | cyphal uses Nunavut (Cyphal-only) | Vendor `dronecan_dsdlc` and wire it into PX4 CMake as a configure/build step (analogous to cyphal's `nnvg`). It produces the legacy-libcanard-compatible C codecs and the optional `cxx_iface` trait. |

---

## 7. Proposed shape of the implementation (starting hypothesis — the next session refines this)

This is *not* the design; it is a strawman to react to.

- **Module:** new `src/drivers/dronecan/` (module name `dronecan`, CLI `dronecan start`), gated by `DRIVERS_DRONECAN`; mutually exclusive with both `DRIVERS_UAVCAN` (old impl) and `DRIVERS_CYPHAL` (decisions 2–3). A board switch is one Kconfig flip.
- **Vendor:** legacy `dronecan/libcanard` + `dronecan_dsdlc` as submodules under the driver. Generate v0 C codecs at build time (CMake configure step, analogous to cyphal's `nnvg`).
- **Transport wrapper:** a `DronecanHandle` modeled on cyphal's `CanardHandle` but driving the *legacy* API (callbacks + internal pool), reusing cyphal's `CanardInterface`/`CanardSocketCAN`/`CanardNuttXCDev` media layer unchanged.
- **Node:** `DronecanNode : px4::ScheduledWorkItem, ModuleParams` on `wq:uavcan`, 3 ms tick + CAN-RX-IRQ wakeup; `transmit();receive();transmit();` drain; `_node_mutex`.
- **Typed pub/sub:** a thin layer over the v0 codecs (see open Q1 — reuse ArduPilot's `Canard::` templates vs a lighter shim; measure for bloat).
- **Bridges:** one subscriber per incoming message → `uORB::PublicationMulti::publish()`, porting the field mappings from the current 15 sensor `.cpp` files. One publisher per outgoing stream.
- **Outputs:** the current `MixingOutput` ESC/servo/hardpoint WorkItems.
- **Services (all preserved, decision 5):** DNA-v0 centralized allocation server, **SD-card firmware/file server** (`BasicFileServer` + `FirmwareUpdateTrigger` + FW DB under `/fs/microsd/ufw`), node monitor → `dronecan_node_status`, time sync, log forward, MAVLink param bridge.
- **Params:** `DC_*` param set + a startup migration function importing legacy `UAVCAN_*` values (decision 4). Boot keyed on `DC_ENABLE` (`rcS`: `if param greater DC_ENABLE 0; then dronecan start`).

---

## 8. Open questions & decisions for the design session

The locked decisions (§2.1) settle vendoring, mutual exclusion, module naming/switch, param rename + migration, and DNA/FW-server scope. Remaining open:

1. **Typed codec layer** — reuse ArduPilot's header-only `Canard::Publisher/Subscriber` templates + `cxx_iface` trait, or a lighter PX4-native shim? These are templates; the whole point is to avoid libuavcan's per-type explosion, so **measure a handful of message types early** to confirm the typed layer stays flat (libcanard's win is the *runtime* C dispatch — keep the C++ veneer minimal). *Highest-leverage remaining question.*
2. **Scope / phasing** — recommended: land a minimal vertical slice first (GNSS + battery + ESC + DNA + SD-card FW server), behind `DRIVERS_DRONECAN`, validated on `ark_fmu-v6x` (fits; real before/after `size`), then fill in the remaining 12 sensor bridges and the aux controllers.
3. **DNA server source** — port ArduPilot's `StorageManager`-backed `AP_DroneCAN_DNA_Server`, or re-host the current `UavcanServers`/`CentralizedServer` behavior on libcanard? Either must preserve allocation **and** SD-card FW serving (decision 5).
4. **MAVLink remote-param bridge & RemoteID** — large, v0-specific subsystems; sequence after the core slice.
5. **Memory model** — legacy libcanard internal-pool sizing vs today's `HeapBasedPoolAllocator` (soft/hard blocks); provide a `dronecan shrink` equivalent?
6. **Param-migration details** — the exact `UAVCAN_*`→`DC_*` name map (mind the 16-char limit); when the migration runs (module start) and its idempotency; whether/when to retire the legacy `UAVCAN_*` definitions; how `rcS` selects `dronecan` vs `uavcan` during the transition.
7. **Validation** — ESC output latency (immediate-flush vs WQ), SITL/HIL DroneCAN, and `size -A` before/after on constrained boards to confirm the ~64 KB lands.

---

## 9. Appendix

### 9.1 Flash measurement methodology (reproducible)
- PX4 buckets: `arm-none-eabi-nm --print-size --radix=d --demangle build/ark_fmu-v6x_default/ark_fmu-v6x_default.elf`, filter ` [TtWwRr] ` text/rodata symbols, dedup by address, bucket by demangled name (`uavcan::` = library; `UavcanNode`/bridges = bridge; `uavcan_stm32h7::` = driver).
- ICF redundancy: relink the existing object set with lld — `arm-none-eabi-g++ ... -B<shim-dir> -Wl,--icf=all -Wl,--print-icf-sections -Wl,--defsym=up_rngaddint=0` where `<shim-dir>/ld` execs `ld.lld` (gcc-10 ignores `-fuse-ld=lld`). Diff `.text` and grep the fold report for uavcan.
- ArduPilot: `cd ardupilot && ./waf configure --board ARKV6X && ./waf copter` → `build/ARKV6X/bin/arducopter`. Buckets: `canard*` (libcanard core), names prefixed by a DroneCAN DSDL namespace `(ardupilot|com|cuav|dronecan|mppt|uavcan)_` (generated codecs), `AP_DroneCAN|CanardInterface|AP_Canard` (driver).

### 9.2 Glossary
- **DroneCAN v0** = UAVCAN v0 = the legacy protocol PX4/ArduPilot use for CAN peripherals. **Cyphal** = UAVCAN v1, a *different, incompatible* protocol (PX4's `cyphal` driver).
- **libcanard (legacy)** = the C library for DroneCAN v0 (callbacks, internal pool). **libcanard (modern)** = OpenCyphal's C library for Cyphal v1 (poll, O1Heap). Same name, different generations, different APIs.
- **DSDL** = the interface-definition language for DroneCAN/Cyphal messages; a code generator turns `.uavcan`/`.dsdl` files into per-type (de)serialization.
- **DNA** = Dynamic Node-ID Allocation (the protocol by which peripherals get a node ID).
- **uORB** = PX4's internal publish/subscribe message bus (the data model the bridges must target).

### 9.3 Memory pointer
Prior flash-reduction findings (ICF measurements, the two failed libuavcan source experiments, the ArduPilot comparison) are recorded in the project memory `project_flash_bloat_icf.md`.
