# DroneCAN libcanard migration — Reference Map (P1)

## 1. Purpose & how to use

This is the **engineering input for designing `src/drivers/dronecan/`** — the consolidated architectural map the P3 design phase engineers from, synthesized from eight reader passes over the three in-repo reference points: ArduPilot `AP_DroneCAN` (behavioral / legacy-libcanard reference), the PX4 `cyphal` driver (PX4-idiom / libcanard-in-PX4 reference), and the current PX4 `uavcan` driver (feature-parity reference). It tells you **which C calls to re-express, which PX4 shapes to copy verbatim, which shells to keep-but-gut, and which features must survive** — at the pointer level. It deliberately does **not** carry the exhaustive per-bridge message→uORB field maps (device-id encoding, redundancy channels, compile/runtime gates, exact field arithmetic); those are the separate **ACCEPTANCE_SPEC** (P2), which is the test oracle for P5/P7 porting. Read this to make structural decisions; read ACCEPTANCE_SPEC to verify a ported bridge. All citations are `path:line` and are preserved verbatim from the readers; ArduPilot root = `/home/jake/code/jake/ardupilot`, PX4 root = `/home/jake/code/jake/PX4-Autopilot`.

---

## 2. The legacy libcanard API surface we must re-express (AP-1, AP-2)

`DronecanHandle` (the new wrapper) must call this **exact** legacy libcanard v0.2 C API — the same library ArduPilot drives and the same one decision 1 vendors. This is a **mutable-instance, callback-driven, single-internal-pool** C library, and it is *source-incompatible* with cyphal's modern libcanard. Do not cross-pollinate.

### 2.1 The exact C calls `DronecanHandle` wraps

| Legacy call | Definition `path:line` | What it does |
|---|---|---|
| `canardInit` | `canard.c:67` (proto `canard.h:462`) | Registers `on_reception` + `should_accept` callbacks, stores `user_reference` (`canard.c:87-91`); slices `mem_arena` into `pool_capacity = mem_arena_size/32` blocks (`canard.c:95`) via `initPoolAllocator` (`canard.c:101`). node_id=0 (anonymous) until `canardSetLocalNodeID`. |
| `canardSetLocalNodeID` | (proto in `canard.h`) | Assigns the local node ID after DNA/param resolution. |
| `canardBroadcastObj` | `canard.c:182` (proto `canard.h:520`) | TX broadcast: builds 29-bit CAN id (anonymous-discriminator path if node_id==0, else normal+CRC), `enqueueTxFrames` into `ins->tx_queue`, and on success calls `incrementTransferID(inout_transfer_id)` (`canard.c:224`). Returns frame count or negative error. |
| `canardRequestOrRespondObj` | `canard.c:355` (proto `canard.h:560`) | TX service: requires node_id!=0 (`CANARD_ERROR_NODE_ID_NOT_SET`); builds service CAN id with dest + req/resp bit. Auto-increments tid **only** for `CanardTransferTypeRequest` (`canard.c:379-382`); responses reuse the request's transfer_id. |
| `canardHandleRxFrame` | `canard.c:403` (proto `canard.h:615`) | RX entry: decodes one frame; on start-of-transfer calls `ins->should_accept` (`canard.c:440`); on a complete transfer calls `ins->on_reception` (`canard.c:530` single-frame, `canard.c:650` multi-frame). |
| `canardPeekTxQueue` / `canardPopTxQueue` | (proto `canard.h`) | TX-drain iteration: peek the queue head, transmit, pop the item back to the pool. |
| `canardCleanupStaleTransfers` | (proto `canard.h`) | Housekeeping: expire stale RX reassembly states; called once per pump pass. |
| `canardGetPoolAllocatorStatistics` | (proto `canard.h`) | Diagnostics: `capacity`/`current`/`peak_usage_blocks` of the single pool. |

POD types passed across the boundary (all in `canard.h`):
- **`CanardInstance`** — owns the pool + callbacks + tx_queue + node_id.
- **`CanardTxTransfer`** (`canard.h:252`) — outgoing-transfer descriptor: `transfer_type`, `data_type_signature`, `data_type_id`, `inout_transfer_id` (pointer to a **persistent** tid var the lib mutates), `priority`, `payload`+`payload_len`, optional `canfd`/`deadline_usec`/`iface_mask`.
- **`CanardRxTransfer`** — completed-transfer view handed to `on_reception`.
- Two callback typedefs: `on_reception` (`canard.h:301`) and `should_accept` (`canard.h:289`).
- `CanardPoolAllocator` / `CanardPoolAllocatorBlock` (struct `canard.h:338`); `CANARD_MEM_BLOCK_SIZE` (`canard.h:117`).

ArduPilot's wrapper threads its C++ object through the C API via `user_reference` and two **static trampolines**, which `DronecanHandle` must re-express identically:
- `canardInit(&canard, mem_arena, size, onTransferReception, shouldAcceptTransfer, this)` — `AP_Canard_iface.cpp:65`/`:64`.
- `onTransferReception` (`AP_Canard_iface.cpp:177`) casts `ins->user_reference` → `CanardInterface*` and calls `handle_message(*transfer)`.
- `shouldAcceptTransfer` (`AP_Canard_iface.cpp:182`) casts back and calls `accept_message(...)` — **MUST write `*out_data_type_signature`** or RX fails CRC.

### 2.2 Contrast table: legacy v0 libcanard (what we keep) vs cyphal's modern libcanard (what we must NOT use)

| Concern | **Legacy v0** (vendor this; AP-1) | **Modern v1** (cyphal; do NOT use) |
|---|---|---|
| Instance | Single mutable `CanardInstance` owns everything | `CanardInstance` + separate `CanardTxQueue` |
| Init | `canardInit(ins, pool, size, on_reception, should_accept, user_ref)` | `canardInit(&alloc, &free)` + `canardTxInit(capacity, mtu)` |
| TX | `canardBroadcastObj` / `canardRequestOrRespondObj` **enqueue** into `ins->tx_queue`, auto-bump caller-owned tid | `canardTxPush()` returns frames you `canardTxPeek`/`canardTxPop` |
| RX | one `canardHandleRxFrame(ins, frame, ts)` → **callbacks** `should_accept`/`on_reception` (push model) | `canardRxAccept()` returns a transfer **by value** (pull model) |
| Subscriptions | implicit — accept decided in the `should_accept` callback | explicit `CanardRxSubscription` objects via `canardRxSubscribe` |
| Memory | **built-in fixed 32-byte block pool** carved from one app-owned arena | external **O1Heap**; `canard_mem_allocate/free` user callbacks; RX payloads + TX frames heap-allocated and app-freed |
| Transfer-id | caller owns a persistent tid var per `(dtid, type, dest)`; lib mutates it | managed inside the library |

Symbol collision (`canard*` defined by both generations) is **resolved by decision 2** — `dronecan` ⊥ `cyphal`, so the two libcanard generations never co-link.

---

## 3. PX4 shapes we reuse verbatim (CY-1, CY-2)

These are the cyphal-proven, version-independent shapes the new driver copies wholesale. The crown jewel is the decode→uORB-publish / uORB-subscribe→encode bridge.

### 3.1 CAN media layer — reuse, re-typed only

`CanardInterface` (`src/drivers/cyphal/CanardInterface.hpp:46`) is a pure-virtual media base — `init()`/`close()`, `transmit(const CanardTxQueueItem&, timeout)` (`:59`), `receive(CanardRxFrame*)` (`:64`) — moving only opaque 11/29-bit-ID CAN frames, with two compile-time backends:
- `CanardSocketCAN` (`CanardSocketCAN.cpp:51/155/180`) — `CONFIG_NET_CAN`, PF_CAN/SOCK_RAW, `SO_TIMESTAMP` RX, `CAN_RAW_TX_DEADLINE` TX, sendmsg/recvmsg.
- `CanardNuttXCDev` (`CanardNuttXCDev.cpp:46/68/115`) — `CONFIG_CAN`, NuttX `/dev/can0`, poll + read/write of `can_msg_s`.

Keep the **shape verbatim** (one abstract base, two compile-time backends, blocking transmit / non-blocking receive); the only change is **re-typing** the frame struct (`CanardTxQueueItem`/`CanardFrame`/`CanardRxFrame` at `CanardInterface.hpp:39`) onto the v0 `CanardCANFrame`. Everything *below* this interface is reusable; everything routed *through* it is opaque bytes. (Carry over the in-source FIXMEs — see Gotchas.)

### 3.2 The bridge pattern (the reusable crown jewel)

**RX (decode → uORB publish)** — `CanardHandle::receive()` (`src/drivers/cyphal/CanardHandle.cpp:111`) loops `_can_interface->receive()` → `canardRxAccept` (`:122`) → on a completed transfer casts `subscription->user_reference` to `UavcanBaseSubscriber*` (`:137`) and calls `sub->callback(receive)` (`:138`). The `user_reference=this` wiring is set in the subscriber ctor (`Subscribers/BaseSubscriber.hpp:60`); the virtual entry point is `callback()` (`BaseSubscriber.hpp:82`). Concrete callbacks then either `*_deserialize_` into a DSDL struct + field-map (hand-written, `Subscribers/udral/Battery.hpp:99`) or — for PX4↔PX4 only — raw-cast the payload (`Subscribers/uORB/uorb_subscriber.hpp:76`), then `uORB::PublicationMulti<T>::publish()` (`uorb_subscriber.hpp:85`).

> **The legacy v0 libcanard shares the same `user_reference` contract on its reception path** — so this exact receive→cast→virtual-callback dispatch shape carries over; only the generated decode symbols differ (and v0 must always go through generated `*_decode`, never a raw cast).

**TX (uORB subscribe → encode)** — symmetric mirror: `uORB::Subscription::updated()/update()` → `*_serialize_` → `CanardHandle::TxPush()` (`uorb_publisher.hpp:63`, `:80`; DSDL example `Publishers/udral/Gnss.hpp:61`/`:89`). `TxPush` is the single TX enqueue funnel (`CanardHandle.cpp:177`).

The dynamic-port registries that drive these are `PublicationManager`/`SubscriptionManager` with compile-time **binder lambda tables** keyed by `CONFIG_CYPHAL_*` (`SubscriptionManager.hpp:130`, e.g. `uORB_over_UAVCAN_Subscriber<sensor_gps_s>` at `:251`); `PublicationManager::update()` (`PublicationManager.cpp:138`) iterates and calls each `dynpub->update()` once per tick.

### 3.3 ScheduledWorkItem scheduling & the Run() drain

`CyphalNode : ModuleParams, ScheduledWorkItem` on `wq:uavcan` (`Cyphal.cpp:69`), `ScheduleOnInterval(3 ms)` (`Cyphal.cpp:117/141`, `ScheduleIntervalMs=3` at `Cyphal.hpp:115`). `Run()` (`Cyphal.cpp:156`): lock `_node_mutex` → lazy `init()` → on `parameter_update` reconfigure → periodic node msgs → `_pub_manager.update()` → **`transmit(); receive(); transmit();`** (`Cyphal.cpp:216-221`) → unlock. The **double-transmit sandwich is load-bearing** — the second flush pushes service responses generated inside RX callbacks out the same tick. A second `OutputModuleInterface` WorkItem (`UavcanMixingInterface`, `Cyphal.hpp:81`) runs actuators on the *same* queue so the two serialize under the shared mutex. **Reuse all of this verbatim**, including the 3 ms base interval and the node mutex.

### 3.4 Codegen / startup wiring

- Codegen *invocation* is part of the seam. Cyphal wires Nunavut `nnvg` at **configure** time via `execute_process` (`CMakeLists.txt:42`/`:45-47`, FATAL on missing). The v0 `uavcan` driver instead uses `add_custom_command` at **build** time with a `.stamp`+`DEPENDS` (`src/drivers/uavcan/CMakeLists.txt:121-130`) → **prefer the build-time custom_command form** for `dronecan_dsdlc` (correct incremental rebuilds).
- Kconfig: mirror `menuconfig DRIVERS_CYPHAL` → `DRIVERS_DRONECAN` with per-bridge `CONFIG_DRONECAN_*` toggles feeding the binder tables + CMake SRCS list (`src/drivers/cyphal/Kconfig`). Note cyphal's toggles reach code **three ways** (Kconfig defines, a CMake `-DCONFIG_CYPHAL_*=1` foreach `CMakeLists.txt:89-101`, and `#ifndef ...0` fallbacks `SubscriptionManager.hpp:44-66`) and the binder-array size is preprocessor-computed (`SubscriptionManager.hpp:70`) — replicate all three layers or the table sizing breaks.
- Startup: `rcS:541-549` makes v0 and Cyphal mutually exclusive (`UAVCAN_ENABLE>0` → `uavcan start`, else `CYPHAL_ENABLE>0` → `cyphal start`). The new driver **takes over the `UAVCAN_ENABLE`/`DC_ENABLE` slot** (replacing the `uavcan` module name) — do **not** add a third branch.

---

## 4. PX4 shapes we rewrite the interior of

The pattern here: **keep the cyphal class shell, replace its guts with the AP-1 legacy API.** Make the boundary explicit.

### 4.1 `DronecanHandle` = cyphal `CanardHandle` shell + legacy guts

**Keep as-is (the shell):** the ownership model and method surface of `CanardHandle` (`src/drivers/cyphal/CanardHandle.hpp:40`) — a single class owning the canard instance + the media interface, exposing a `receive()` pump, a `transmit()` pump, a `TxPush`, subscription register/unregister, and `node_id`/`mtu`/diagnostics accessors. This is the integration seam `Cyphal.cpp`/`DronecanNode` expects.

**Rewrite the interior** of ctor / `receive()` / `transmit()` / `TxPush` / `RxSubscribe` to the legacy v0 API (AP-1):

| Method | Cyphal v1 interior (replace) | Legacy v0 interior (write) |
|---|---|---|
| ctor | `o1heapInit` + `canardInit(&alloc,&free)` + `canardTxInit` (`CanardHandle.cpp:63-86`) | `canardInit(ins, pool, size, on_reception, should_accept, this)` over a statically-sized internal pool |
| `receive()` | `canardRxAccept` poll loop + app-side `memory_free` (`CanardHandle.cpp:111-152`) | `canardHandleRxFrame` push model; dispatch happens **inside** the `on_reception` callback (no app-side free) |
| `transmit()` | `canardTxPeek`/`canardTxPop` drain + free (`CanardHandle.cpp:154-175`) | `canardPeekTxQueue`/`canardPopTxQueue` drain |
| `TxPush` | `canardTxPush` (`CanardHandle.cpp:177`) | `canardBroadcastObj` / `canardRequestOrRespondObj` |
| `RxSubscribe` | per-port `canardRxSubscribe` (`CanardHandle.cpp:185`) | no per-port subscribe; accept decided in `should_accept` against the handler registry |

**Boundary statement:** the class *shape* survives, the libcanard-touching *bodies* are all replaced. The O1Heap arena + `memAllocate`/`memFree` + 8 KB `HeapSize` (`CanardHandle.cpp:57-66`, `.hpp:48`) are **dropped entirely** (see §6, do-not-copy) in favor of a v0 internal pool.

### 4.2 ArduPilot's `CanardInterface` wrapper = structural template for the rewrite

ArduPilot's `CanardInterface` (`AP_Canard_iface.h`/`.cpp`) is the *behavioral* template for that rewritten interior — keep its structural pattern, swap its HAL:

| AP structural element | `path:line` | Disposition |
|---|---|---|
| Own the `CanardInstance`, register two static trampolines, thread `this` via `user_reference`, hold a scratch `CanardTxTransfer`, expose broadcast/request/respond + process/processTx/processRx | `AP_Canard_iface.h/.cpp` | **rewrite-interior** — keep the structure, replace `AP_HAL::CANIface` select/send/receive + `HAL_Semaphore`/`WITH_SEMAPHORE` with PX4 CAN device + PX4 locking |
| Static trampolines `onTransferReception`/`shouldAcceptTransfer` casting `user_reference` | `AP_Canard_iface.cpp:177,182` | **port-mapping** — re-express identically; `accept_message` must set the out-signature; `on_reception` dispatches to PX4's handler registry |
| `processTx` drain incl. `raw_commands_only` ESC filter + iface_mask/deadline | `AP_Canard_iface.cpp:206-283` | **rewrite-interior** — keep the behavior (a TX-drain with an immediate ESC-only low-latency variant); AP's multi-iface/iface_mask is optional for PX4 single-iface |
| Immediate-flush `processTx(true)` after ESC broadcast | `AP_DroneCAN.cpp:860,913` | **port-mapping** — preserve the semantic: flush TX synchronously from the actuator publish path (a `DronecanHandle::flush()`), don't wait for the next tick |
| `process()` open `while(true)` pump + dedicated CAN thread + `sem_handle.wait` | `AP_Canard_iface.cpp:391`, `AP_DroneCAN.cpp:511` | **do-not-copy** — take only the per-iteration body (`processRx → processTx → canardCleanupStaleTransfers`) and run it once per `ScheduledWorkItem` tick |

---

## 5. Feature inventory to preserve (UA-1, UA-2, UA-3)

Pointer-level only — the per-field message→uORB maps live in **ACCEPTANCE_SPEC (P2)**. Each item tagged by phase (P4 vertical slice / P5 remaining bridges / P6 heavy subsystems), per PLAN.md.

### 5.1 Node lifecycle / scheduling / CLI / memory / params (UA-1) — P4

- **`UavcanNode : ScheduledWorkItem, ModuleParams`** singleton on `wq:uavcan` (`uavcan_main.hpp:181`); **dual scheduling**: `ScheduleOnInterval(3 ms)` (`uavcan_main.cpp:451`) **plus** CAN-RX-IRQ `ScheduleNow()` via `busevent_signal_trampoline` (`uavcan_main.cpp:486-490`, registered `:497`; IRQ source `uc_stm32h7_thread.cpp:54`). → **P4.**
- **CLI surface** `uavcan {start|status|stop|shrink|update|param {set|get|list|save} <node>|reset <node>}` (`uavcan_main.cpp:1462/1469`). Rename binary to `dronecan`. `shrink` may become a no-op given the v0 internal pool; `update` depends on porting the FW server. → **P4** (CLI), see ACCEPTANCE_SPEC / §8 for behavior.
- **`UAVCAN_ENABLE` 0/1/2/3 tiers** — 0=off; ≥1 sensors; >1 enables `UavcanServers` (DNA+FW); >2 enables ESC/actuator outputs (gates at `uavcan_main.cpp:536/621`, `module.yaml show_subgroups_if UAVCAN_ENABLE>=3`, `rcS:541`). Rename `UAVCAN_*`→`DC_*` (decision 4), preserve all three thresholds. → **P4** (gating) / **P6** (migration fn).
- **`wq:uavcan` config** (stack 3754, prio -19; `WorkQueueManager.hpp:82`, `px4_work_queue/Kconfig:208/215`) + the 3 ms interval — reuse the slot as-is (stack may shrink once libuavcan templates are gone). → **P4.**

### 5.2 Services & node management (UA-2)

- **DNA-v0 centralized server** — `UavcanServers` bundles `CentralizedServer` persisting allocations to `/fs/microsd/uavcan.db` (`uavcan_servers.cpp:130`, gated `UAVCAN_ENABLE>1`). The PX4 node's **own** id is a static `UAVCAN_NODE_ID` param, *not* DNA-allocated (`uavcan_main.cpp:682-688`). → **P4.** ⇒ ACCEPTANCE_SPEC for the persistence record format.
- **SD-card firmware server** — `BasicFileServer` + `FirmwareUpdateTrigger` + `FirmwareVersionChecker` + plaintext `FW.db` over `/fs/microsd/ufw`, ingesting user `.bin` from SD root + `ufw_staging` (`uavcan_servers.cpp:83/138/163/300/352`). POSIX FW.db/copy/migrate/validate helpers (`uavcan_servers.cpp:245-407`) port nearly verbatim; transport + version-trigger rebuilt on libcanard. Decision 5 = **in scope**. → **P4.**
- **NodeInfoRetriever** — auto-`GetNodeInfo` on new peers; central dependency of NodeInfoPublisher, FW trigger, active-node iteration (`uavcan_main.hpp:294`/`uavcan_main.cpp:591`). **No libcanard equivalent — hand-roll.** → **P4.**
- **NodeStatusMonitor → `dronecan_node_status`** (`uavcan_main.cpp:1032`, 100 ms, lazy NodeID↔uORB-multi-instance map). → **P4.** ⇒ ACCEPTANCE_SPEC.
- **NodeInfoPublisher → `device_information`** (`node_info.cpp:43/194`, join GetNodeInfo + sensor-bridge capability registrations, round-robin 1 Hz). → **P4.** ⇒ ACCEPTANCE_SPEC (DeviceCapability map, field formats).
- **`can_interface_status`** (`uavcan_main.cpp:994`, 100 ms; source counters from the libcanard socketcan/CAN layer instead of `getCanIOManager()`). → **P4.**
- **GlobalTimeSync master+slave**, NodeID-priority arbitration, 1 Hz (`uavcan_main.cpp:637`). No libcanard master/slave class — implement the previous-transmission-timestamp protocol manually. → **P4/P6.**
- **`debug.LogMessage → px4_log`** (`logmessage.hpp:64`, tag `uavcan:<node>:<source>`). Trivial. → **P4** (small).
- **MAVLink remote-param bridge** — single-flight FSM in `Run()` (`uavcan_main.cpp:771-970`) over GetSet/ExecuteOpcode/RestartNode; `_param_counts[128]`, `_param_dirty_bitmap[4]`, count-first + dirty-bitmap save/restart. → **P6.** ⇒ requires manual transfer-id↔node-id correlation (see §8 Q4).

### 5.3 Actuator outputs & aux controllers (UA-3)

- **ESC** (P4) — `UavcanEscController`: out `esc.RawCommand` ≤400 Hz; in `esc.Status`/`StatusExtended` → `esc_status` with 1200 ms freshness + node-health/VertiQ failure parsing (`actuators/esc.cpp:88/113/143/155/171`). → **P4.** ⇒ ACCEPTANCE_SPEC for field maps; failure parsing depends on `dronecan_node_status` + `device_information` uORB pipeline.
- **Servo** — `UavcanServoController`: `actuator.ArrayCommand`, `command_value=outputs[i]/500-1`, COMMAND_TYPE_UNITLESS, no internal rate gate (`actuators/servo.cpp:47`). → **P5.**
- **Hardpoint** — `hardpoint.Command`, 10 Hz + 1 Hz keepalive, `DO_GRIPPER` (`actuators/hardpoint.cpp:69`). → **P5.**
- **MixingOutput WorkItem skeleton** — `UavcanMixingInterfaceESC`/`Servo` (`OutputModuleInterface`, `uavcan_main.hpp:124,156`), `"UAVCAN_EC"`/`"UAVCAN_SV"`, `Run/updateOutputs/mixerChanged` (`uavcan_main.cpp:1093-1150`), shared `_node_mutex`. **Reuse-as-is** (§6.5 decision); only the inner `update_outputs` encode changes. → **P4/P5.**
- **Aux controllers** (uORB→DSDL bridges on libuavcan periodic timers, each `CONFIG_UAVCAN_*`-gated): ArmingStatus 10 Hz (`arming_status.cpp:62`), SafetyState 10 Hz (`safety_state.cpp:62`, **ardupilot** vendor DSDL), Beep 100 Hz (`beep.cpp:69`, CBRK_BUZZER), RGBLED 20 Hz (`rgbled.cpp:86`, RGB565). → **P5** (mechanical port-mapping).
- **RemoteID** — bidirectional Open Drone ID, 1 Hz, 5 broadcasts + ArmStatus rx (`remoteid.cpp:70`); pulls `modules/mavlink/open_drone_id_translations.hpp`. → **P6** (deferred; heaviest aux, **dronecan** vendor DSDL).
- **GNSS RTCM/MovingBaseline uplink + PPK `gps_dump`** — physically in `sensors/gnss.cpp:362/572/652/682`, ported within the **GNSS sensor-bridge** work, not the actuator driver. → **P5.**

---

## 6. Consolidated reuse/rewrite/port disposition table

Every `reuseDisposition` across all eight slices, merged. Dispositions: **reuse-as-is** (copy unchanged), **rewrite-interior** (keep shell/shape, replace guts), **port-mapping** (re-express behavior on new API), **reference-only** (read for behavior, don't copy code), **do-not-copy** (deliberately drop).

| Element | Source | Disposition | Note |
|---|---|---|---|
| Legacy libcanard C API (canardInit / Broadcast / RequestOrRespond / HandleRxFrame / Peek/Pop / Cleanup + POD types) | AP-1 `canard.h`/`canard.c` | **reuse-as-is** | This *is* the API the new driver keeps; vendor the same libcanard. Do NOT swap to cyphal's `canardTxPush`/`canardRxAccept`. |
| Single shared 32-byte block pool (one `mem_arena` → free-list serving BOTH RX states + TX items; `allocateBlock`/`freeBlock`) | AP-1 `canard.c:1891/1915/1945` | **reuse-as-is** | Inherited by keeping libcanard. PX4 owns the arena, passes a sized **word-aligned** byte region (mirror AP's POOL 8192/16384). Surface `peak_usage_blocks`. |
| `canardBroadcast`/`canardRequestOrRespond` legacy non-`Obj` variants | AP-1 `canard.c:142/314` | **reference-only** | Header-marked legacy; AP uses only `*Obj`. PX4 should too. |
| `CanardInterface` wrapper overall shape (own instance, 2 trampolines, scratch tx_transfer, broadcast/request/respond + process/processTx/processRx) | AP-1 `AP_Canard_iface.h/.cpp` | **rewrite-interior** | Keep the structural pattern; replace AP HAL + semaphores with PX4 CAN device + PX4 locking. |
| Static trampolines `onTransferReception`/`shouldAcceptTransfer` (cast `user_reference`) | AP-1 `AP_Canard_iface.cpp:177,182` | **port-mapping** | Re-express identically; `accept_message` MUST set out-signature; `on_reception` → PX4 handler registry. |
| `processTx` drain incl. ESC `raw_commands_only` filter + iface_mask/deadline | AP-1 `AP_Canard_iface.cpp:206-283` | **rewrite-interior** | Keep TX-drain + immediate ESC-only flush behavior; rewrite per-frame send on PX4 CAN tx; iface_mask optional. |
| Immediate-flush `processTx(true)` after ESC broadcast | AP-1/AP-3 `AP_DroneCAN.cpp:860,913` | **port-mapping** | Flush TX synchronously from actuator publish path; map to `DronecanHandle::flush()`. |
| `process()` while(true) pump + dedicated CAN thread + `sem_handle.wait` | AP-1 `AP_Canard_iface.cpp:391`, `AP_DroneCAN.cpp:511` | **do-not-copy** | Take only the per-iteration body; drive from a `ScheduledWorkItem` Run(), drop the loop + event semaphore. |
| AP extras: `add_11bit_driver`/`write_aux_frame`, test loopback iface, `dronecan_protocol_Stats`, `CANARD_MULTI_IFACE`/iface_mask, `CANARD_ALLOCATE_SEM` hooks | AP-1 | **do-not-copy** | AP HAL plumbing + test scaffolding. `CANARD_ALLOCATE_SEM=0` is fine for PX4's single-WorkItem model. |
| `cxx_iface` trait concept (static ID/SIGNATURE/MAX_SIZE + encode/decode) | AP-2 `cxx_wrappers.h:32/56` | **port-mapping** | Keep the **seam** (the highest-leverage reuse — decouples codecs from dispatch). Shrink it: prefer plain `static constexpr` values + direct calls over fn-ptr **members** (fn-ptrs block inlining = bloat). Mind the TAO/CANFD encode-arity split. |
| `Publisher<T>`/`Subscriber<T>`/`Server<T>`/`Client<T>` templates | AP-2 `publisher.h`/`subscriber.h`/`service_*.h` | **reference-only** | Behaviorally exactly right but each instantiation = a fresh `msg_buf[MAX_SIZE]` + encode/decode body + vtable = the libuavcan-class bloat we're escaping. Reuse the **behavior**, not the template bodies; prefer a non-templated descriptor-driven dispatcher. |
| `HandlerList head[CANARD_NUM_HANDLERS][8]` dispatch + `link`/`unlink` self-registration | AP-2/AP-3 `handler_list.h:67-119/147` | **port-mapping** | Replicate the allocation-free hashed `[index][msgid%8]` dispatch + non-broadcast stop-after-first, but reimplement **per-instance** inside the PX4 driver object, not via static-global `head[][]` + `DEFINE_*` singletons. |
| `Callback<T>` virtual hierarchy + `allocate_sub_*_callback` heap helpers + `TransferObject` malloc'd tid map | AP-2 `callbacks.h`, `subscriber.h:90-134`, `transfer_object.h` | **do-not-copy** | Virtual dispatch + malloc-per-subscription/tid are antithetical to PX4 static allocation. Use statically-constructed handlers + a fixed/descriptor-keyed tid store. Keep only the `MAKE_TRANSFER_DESCRIPTOR` (id\|type\|src\|dst) concept. |
| `dronecan_dsdlc` generator + plain-C struct/encode/decode/#define output contract | AP-2 `dronecan_dsdlc.py`/`_helpers.py` + templates | **reuse-as-is** | Reuse essentially verbatim — libcanard-native, battle-tested across 200+ types, decoupled from the C++ veneer. Run with `DRONECAN_CXX_WRAPPERS` controlling only whether the trait is emitted; a lighter PX4 shim can consume raw `*_encode`/`*_decode` + `_ID`/`_SIGNATURE`/`_MAX_SIZE` directly. |
| DNA two-step UID allocation FSM + `Database::handle_allocation` | AP-3 `AP_DroneCAN_DNA_Server.cpp:119/506` | **port-mapping** | Reproduce: ignore non-zero src; reset on follow-up timeout AND every first_part; accumulate UID chunks; assign free id scanning down from 125; re-broadcast; never CAN-FD the Allocation msg. Do NOT mix node-id into the UID (AP wart). |
| Node-ID↔UID persistence (NodeRecord 6-byte FNV-1a + CRC8, `Bitmask<128>`, StorageCANDNA 1024 B) | AP-3 `AP_DroneCAN_DNA_Server.cpp` / `StorageManager.cpp:88/95` | **port-mapping** | Persist a node-ID-indexed table + in-RAM presence bitmask; magic-validation + reset-on-mismatch. PX4 may store raw 16-byte UID if storage allows (AP folds to 6 B only to fit 1024 B). |
| Node monitoring (reactive GetNodeInfo on NodeStatus + 5 s `verify_nodes` sweep + duplicate detect) | AP-3 `AP_DroneCAN_DNA_Server.cpp:286/410/447` | **port-mapping** | Safety-relevant prearm signal. Reproduce duplicate/dropout detection; exclude own node ID. |
| `UavcanServers` CentralizedServer DNA + `/fs/microsd/uavcan.db` persistence | UA-2 `uavcan_servers.cpp:130` | **port-mapping** | P4 core. Keep behavior; the v0 `FileStorageBackend`/`FileEventTracer` **file format** is libuavcan-specific → do-not-copy the format, only the persistence behavior. |
| SD FW server: `BasicFileServer`+`FirmwareUpdateTrigger`+`FirmwareVersionChecker`+`FW.db`+migrate/validate | UA-2 `uavcan_servers.cpp:83-407` | **port-mapping** | P4 core (decision 5). POSIX FW.db/copy/migrate/validate (`:245-407`) port almost verbatim; file-server transport + version-trigger rebuilt on libcanard. |
| `NodeInfoRetriever` | UA-2 `uavcan_main.cpp:591` | **port-mapping** | P4 core. No libcanard equivalent — hand-roll GetNodeInfo client with `invalidateAll()`/`isNodeKnown()` + listener fan-out. |
| `NodeStatusMonitor → dronecan_node_status` (+ NodeID↔uORB-index map) | UA-2 `uavcan_main.cpp:1032` | **port-mapping** | P4 core. uORB schema unchanged. |
| `NodeInfoPublisher → device_information` | UA-2 `node_info.cpp` | **port-mapping** | P4 core. Keep join-two-sources + DeviceCapability→DEVICE_TYPE_* map + field-format strings; only the v0 `INodeInfoListener`/`TimerBase` hooks change. |
| `can_interface_status` publishing | UA-2 `uavcan_main.cpp:994` | **port-mapping** | P4 core. Source counters from the libcanard socketcan/CAN layer. |
| GlobalTimeSync master+slave (NodeID-priority) | UA-2 `uavcan_main.cpp:637` | **port-mapping** | P4 core. Implement previous-transmission-timestamp protocol manually; preserve adjustUtc/hrt seeding. |
| `debug.LogMessage → px4_log` | UA-2/UA-3 `logmessage.hpp:64` | **port-mapping** | Small/trivial. Always-on (no CONFIG gate). |
| MAVLink remote-param bridge (GetSet/ExecuteOpcode/RestartNode SM + uORB) | UA-2 `uavcan_main.cpp:771-970` + callbacks | **port-mapping** | **P6.** Port single-flight FSM + `_param_counts` cache + dirty bitmap; uORB contract unchanged. Needs manual transfer-id↔node-id correlation (Q4). |
| Blocking CLI param ops (transient ServiceClient + usleep busy-wait) | UA-1 `uavcan_main.cpp:257/393/...` | **rewrite-interior** | Same UX; reimplement synchronous call/wait over libcanard service transfers; the `usleep(10000)` poll idiom is libuavcan-specific. |
| `busevent_signal_trampoline` + `registerSignalCallback` | UA-1 `uavcan_main.cpp:486-497` | **rewrite-interior** | Keep behavior (IRQ → `ScheduleNow()` on the singleton); replace the libuavcan BusEvent plumbing with the new CAN HAL RX callback. |
| `UavcanNode : ScheduledWorkItem+ModuleParams` singleton on `wq:uavcan` | UA-1 `uavcan_main.hpp:181` | **port-mapping** | Feature-parity bedrock — reproduce the exact shape. |
| Dual scheduling: `ScheduleOnInterval(3 ms)` + CAN-RX-IRQ `ScheduleNow()` | UA-1 `uavcan_main.cpp:451/486-490` | **port-mapping** | Keep both: 3 ms bounds latency/timeouts; IRQ gives low-latency RX. Wire `ScheduleNow()` from the CAN driver RX IRQ. |
| CLI surface `uavcan {start\|status\|stop\|shrink\|update\|param ...}` | UA-1 `uavcan_main.cpp:1462/1469` | **port-mapping** | Reproduce verbatim (rename binary `dronecan`). `shrink` may alias to no-op; `update` depends on FW server. |
| HeapBasedPoolAllocator block 48, soft 250/hard 500, `shrink()` | UA-1 `allocator.hpp`, `CMakeLists.txt:84` | **reference-only** | Q5: do NOT copy libuavcan's allocator — use these numbers only to size the v0 pool budget and decide if `shrink` stays meaningful. |
| Remote-node param get/set/list/save state machine (cb_getset/count/opcode/restart, dirty bitmap) | UA-1 `uavcan_main.cpp:1250-1457` | **port-mapping** | Reproduce over libcanard GetSet/ExecuteOpcode/RestartNode clients; count-first + dirty-bitmap sequencing is the contract MAVLink GCSs depend on. |
| `UAVCAN_*` param names → `DC_*` | UA-1/UA-2 | **port-mapping** | Decision 4. Preserve semantics: ENABLE tiers, NODE_ID 1-125, BITRATE, ESC_IFACE bitmask, SUB_*/PUB_*, EC*/SV* groups. |
| `UAVCAN_ENABLE` 0/1/2/3 tier semantics + rcS keying | UA-1/UA-2 `uavcan_main.cpp:536/621`, `rcS:541` | **port-mapping** | Reproduce all three thresholds; preserve mutual-exclusion-with-cyphal else-branch. |
| `_servers` lifecycle (heap new in init, delete in dtor) | UA-2 `uavcan_main.cpp:621` | **reference-only** | Pattern only — optional services module created after node up, torn down with it; rewrite internals. |
| `UavcanMixingInterfaceESC/Servo` + `MixingOutput` WorkItem skeleton | UA-3 `uavcan_main.hpp:124,156`/`uavcan_main.cpp:1093-1150` | **reuse-as-is** | §6.5. Keep Run/updateOutputs/mixerChanged, `"UAVCAN_EC"`/`"UAVCAN_SV"` prefixes, output_array_size bandwidth trim, `setMaxTopicUpdateRate`, ScheduleNow, `_node_mutex`. Only inner encode changes. |
| `UavcanEscController` interior (RawCommand encode/rate-gate + Status decode + esc_status aggregation + failure parsing) | UA-3 `actuators/esc.cpp` | **rewrite-interior** | P4. Keep esc_status field map, 1200 ms freshness, VertiQ/iq_motion failure table as pure functions; replace libuavcan Publisher/Subscriber + `_node.getMonotonicTime` rate gate with canard encode/decode + `hrt_absolute_time`. |
| Servo `/500-1` unitless mapping | UA-3 `servo.cpp:47` | **port-mapping** | Preserve `command_value=outputs[i]/500-1` + COMMAND_TYPE_UNITLESS for parity (source TODO wants [-1,1] but don't change behavior silently). |
| `UavcanHardpointController` | UA-3 `hardpoint.cpp:69` | **port-mapping** | Replace libuavcan timer; keep DO_GRIPPER→Command, 10 Hz max + 1 Hz keepalive. |
| ArmingStatus / SafetyState / Beep / RGBLED | UA-3 | **port-mapping** | Pure uORB→DSDL bridges; re-host on new timer/work mechanism. Keep rates (10/10/100/20 Hz), gates, Tunes SM, RGB565 conv. SafetyState uses **ardupilot** vendor DSDL — must generate. |
| `UavcanLogMessage` | UA-3 `logmessage.hpp:64` | **port-mapping** | Trivial; always-on. |
| `UavcanRemoteIDController` | UA-3 `remoteid.cpp:70` | **port-mapping** | **Flag P6.** Large, **dronecan** vendor-DSDL-specific, depends on `open_drone_id_translations.hpp`; defer until core/sensor slices land. |
| GNSS RTCM/MovingBaseline injection + PPK `gps_dump` | UA-3 `sensors/gnss.cpp:362/572/652/682` | **reference-only** (here) | Belongs to the GNSS sensor-bridge slice; port within that work, not the actuator driver. |
| AP `Canard::Subscriber/Publisher/HandlerList/ObjCallback` dispatch machinery | AP-3 | **do-not-copy** | AP's C++ wrapper layer; PX4 builds its own dispatch. Only the resulting behavior is the reference. |
| libcanard `canard.c` reassembly engine (`canardHandleRxFrame`, single/multi-frame + CRC completion) | AP-3 `canard.c:403/530/648-650` | **reuse-as-is** | Upstream libcanard — the new driver uses the same library; RX completion points + should_accept/on_reception contract carry over unchanged. |
| Trace 1/2/3 end-to-end pipelines (Fix2 decode→backend-resolve→state-fill; ESC flush; in-node esc.Status) | AP-3 | **reference-only** | Behavioral templates for the PX4 decode→uORB-publish / uORB-subscribe→encode bridges (PX4 publishes uORB keyed by source node-ID instead of filling GPS_State). |
| **— cyphal side —** | | | |
| `CanardInterface` media abstraction (init/close/transmit/receive) | CY-1 `CanardInterface.hpp:46` | **port-mapping** | Keep the SHAPE verbatim (one abstract base, two compile-time backends, blocking transmit/non-blocking receive); **re-type** the carried frame struct onto v0 `CanardCANFrame`. Explicit as-is boundary: below it reusable, through it opaque bytes. |
| `CanardSocketCAN` (CONFIG_NET_CAN) | CY-1 `CanardSocketCAN.cpp` | **reuse-as-is** | Pure OS/wire plumbing, ZERO protocol knowledge; adapt only the frame-field reads. Carry over the FIXMEs. |
| `CanardNuttXCDev` (CONFIG_CAN) | CY-1 `CanardNuttXCDev.cpp` | **reuse-as-is** | NuttX char-device plumbing; same adapt-the-frame-struct treatment. Note board-specific `stm32_can.h`/instance(1). |
| `CanardRxFrame` struct | CY-1 `CanardInterface.hpp:39` | **port-mapping** | Trivial `{timestamp_usec, frame}` carrier; re-type frame to v0 struct. |
| `CanardHandle` SHELL (owns instance+queue+heap+interface; receive()/transmit() pumps; TxPush/RxSubscribe; accessors) | CY-1 `CanardHandle.hpp:40` | **reuse-as-is** | Keep ownership model + method surface — the integration seam. |
| `CanardHandle` INTERIOR (modern-API bodies + O1Heap) | CY-1 `CanardHandle.cpp:63-198` | **rewrite-interior** | Rewrite to the legacy v0 API (see §4); only the surrounding class shape survives. |
| O1Heap arena + memAllocate/memFree + 8 KB HeapSize | CY-1 `CanardHandle.cpp:57-66`, `.hpp:48` | **do-not-copy** | Q5. Drop O1Heap entirely; v0 manages a fixed internal pool. Keep the diagnostics-accessor *idea* but source from v0 pool stats. |
| `Cyphal::Run()` cadence (transmit→receive→transmit) | CY-1/CY-2 `Cyphal.cpp:216-221` | **reuse-as-is** | Keep the exact ordering; the double-transmit flushes same-tick responses. |
| RX dispatch shape (canardRxAccept → user_reference cast → virtual callback) | CY-2 `CanardHandle.cpp:111-152` | **reuse-as-is** | Version-independent (v0 has the same `user_reference` contract); only generated type symbols differ. |
| Subscriber base + virtual callback() + `user_reference=this` wiring | CY-2 `BaseSubscriber.hpp:60/82` | **reuse-as-is** | What makes RX dispatch O(1) and polymorphic. |
| Generic `uORB_over_UAVCAN_Subscriber<T>` decode→PublicationMulti::publish() | CY-2 `uorb_subscriber.hpp` | **reuse-as-is** (shape) | For DroneCAN, deserialize via generated `*_decode` instead of the raw cast (raw cast is PX4↔PX4 only). |
| Hand-written multi-subject bridge (per-port_id branch → deserialize → field map → publish) | CY-2 `Battery.hpp` | **rewrite-interior** | Keep the SHAPE; rewrite the body — v0 DSDL types/field names differ. |
| Generic `uORB_over_UAVCAN_Publisher<T>::update()` → serialize → TxPush | CY-2 `uorb_publisher.hpp`+`Gnss.hpp` | **reuse-as-is** (shape) | Always serialize via generated `*_encode` for v0 (wire format not struct-blittable). |
| `UavcanPublisher`/`UavcanDynamicPortSubscriber` + `updateParam()` register→port-ID binding | CY-2 `Publisher.hpp:79`, `DynamicPortSubscriber.hpp:61` | **reference-only** | v0 uses **fixed** data-type IDs — the whole register/port-ID layer collapses; bind statically. Do NOT port. |
| `PublicationManager`/`SubscriptionManager` registries + binder lambda tables | CY-2 `SubscriptionManager.hpp:130` | **port-mapping** | Keep the registry concept (config-toggle→factory, iterate update()); simplify (no dynamic port-ID). Maps to `CONFIG_DRONECAN_*`. |
| `ScheduledWorkItem` on `wq:uavcan` + `ScheduleOnInterval(3 ms)` + `_node_mutex` | CY-2 `Cyphal.hpp/.cpp` | **reuse-as-is** | Same scheduling model; keep 3 ms + node mutex + optional 2nd same-queue WorkItem. |
| `UavcanMixingInterface` as a separate `OutputModuleInterface` WorkItem on the same wq | CY-2 `Cyphal.hpp:81` | **reuse-as-is** | The two-WorkItem split (node fixed-rate, actuators on MixingOutput updates) is right for v0 too. |
| Nunavut `nnvg` codegen via `execute_process` at configure time | CY-2 `CMakeLists.txt:42-50` | **port-mapping** | Swap nnvg → `dronecan_dsdlc`; prefer the v0 driver's **build-time** `add_custom_command` form (`uavcan/CMakeLists.txt:121-130`) for incremental rebuilds. |
| Kconfig `menuconfig DRIVERS_CYPHAL` + per-bridge `CONFIG_CYPHAL_*` | CY-2 `Kconfig` | **port-mapping** | Mirror as `DRIVERS_DRONECAN` + `CONFIG_DRONECAN_*` feeding binder tables + SRCS. |
| rcS mutual-exclusion block | CY-2/UA-1 `rcS:541-549` | **reference-only** | The new driver REPLACES the v0 `uavcan` module behind `UAVCAN_ENABLE`/`DC_ENABLE`; don't add a 3rd branch. |

---

## 7. Gotchas (merged + deduped)

**LOUD — the three you must not miss:**

1. **`dronecan_dsdlc` `<type>_decode` returns `TRUE` on FAILURE / `false` on success.** (`msg.c.em:32-34`; e.g. `air_data.StaticPressure.h:53-54`.) Every consumer must negate — AP guards with `if (!cxx_iface::decode(...))` (`subscriber.h:61`, `service_server.h:59`, `service_client.h:63`). A PX4 shim that forgets the negation **silently processes garbage / drops valid frames.** Conversely `encode()` returns **BYTE LENGTH** (0 == failure, check `len > 0`). `decode` also does an EXACT-length check by default (`byte_len != payload_len`, `msg.c.em:59`); only `CANARD_ENABLE_TAO_OPTION` with `!tao` relaxes it to `>`.

2. **Legacy single internal pool — NOT O1Heap.** There is exactly ONE arena. `createTxItem` (`canard.c:1338`) and `createRxState` (`canard.c:1513`) **both** call `allocateBlock()` on `ins->allocator`. A burst of RX reassembly can starve TX enqueue and vice-versa; `canardBroadcastObj` returning ≤0 may mean OOM, not a full bus. Block size = 32 B default (`CANARD_MEM_BLOCK_SIZE`, `canard.h:117`), 40 with `CANARD_ENABLE_DEADLINE`, 128 with `CANARD_ENABLE_CANFD`; `pool_capacity = arena/block` (`canard.c:95`). The arena **must be word-aligned** (AP passes a `uint32_t[]`, `AP_DroneCAN.cpp:345`). Size generously (AP defaults 8192 non-FD / 16384 FD) and monitor `peak_usage_blocks`. *This is fully distinct from cyphal's O1Heap (8 KB arbitrary-extent arena, `CanardHandle.hpp:42-48`) — the 8 KB number does NOT port 1:1.*

3. **The `canard*` symbol collision is real but RESOLVED by decision 2.** The two libcanard generations define the same `canard*` symbols. `dronecan` ⊥ `cyphal` (Kconfig `DRIVERS_CYPHAL depends on !DRIVERS_DRONECAN` + CMake warning) means they never co-link, so the duplicate symbols never meet. Do not attempt to make them coexist in one binary.

**Legacy API contract:**
- `should_accept` is contractual: returning true **MUST** write `*out_data_type_signature` or RX fails CRC (`canard.h:284`). It is also called a SECOND time on non-start frames that have no RX state (`canard.c:465`) to cheaply reject mid-transfer frames — so keep it cheap and side-effect-free.
- Transfer-id ownership: `inout_transfer_id` MUST point to a **persistent** (static/heap) var, never the stack, and must NOT be shared across different `(data_type_id, transfer_type, dest)` descriptors — the lib mutates it (`canard.c:224/381`). **Responses deliberately do NOT increment the tid** (`canard.c:379` guards on `CanardTransferTypeRequest`); a response reuses the matching request's transfer_id.
- Anonymous-node TX restriction: if node_id==0, `canardBroadcastObj` rejects payloads >7 bytes and limits dtid to the ANON range (`canard.c:200`); `canardRequestOrRespondObj` fails entirely (`canard.c:365`). Relevant before a node ID is assigned.
- AP's wrapper reuses ONE member `CanardTxTransfer` (`AP_Canard_iface.h:83`) overwritten by every broadcast/request/respond — hence its `_sem_tx`. A PX4 re-impl that builds the `CanardTxTransfer` **on the stack per-call** avoids this shared-state hazard.

**Scheduling / threading (the trap):**
- ArduPilot runs a DEDICATED CAN thread with an open `while(true)` + blocking `sem_handle.wait` (`AP_Canard_iface.cpp:400-414`, `AP_DroneCAN.cpp:511-523`). PX4 must run the per-tick body from a `ScheduledWorkItem` — do NOT port the loop or the event semaphore. Without AP's semaphore-gated wait, size the WorkItem period to bound TX latency; `processTx(true)` is what compensates on the ESC path.
- The immediate-flush `processTx(true)` filters to ESC RawCommand IDs only (`UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID` / `COM_HOBBYWING_ESC_RAWCOMMAND_ID`, `AP_Canard_iface.cpp:234-236`) and walks past (does not transmit) every non-ESC queued frame on that pass. The ESC send path runs in the **vehicle output thread** via `SRV_Channels::push` (`SRV_Channels.cpp:530`), NOT the CAN loop — so the TX queue is touched from two threads (AP guards both with `_sem_tx`). If PX4 flushes ESC synchronously from the output thread, preserve cross-thread locking.
- Concurrency: AP guards TX/RX with separate `_sem_tx`/`_sem_rx`, holding BOTH for `canardCleanupStaleTransfers` (`AP_Canard_iface.cpp:404-406`). libcanard itself is NOT internally locked unless built with `CANARD_ALLOCATE_SEM=1` (`AP_Canard_iface.cpp:30-49`). If PX4 keeps everything in one WorkItem, no locks are needed and `CANARD_ALLOCATE_SEM` can stay 0; **if CAN RX is interrupt/threaded, you MUST add locking around the shared pool.** PX4's whole `Run()` (and the mixing WorkItem) is already serialized by a single `_node_mutex` — preserve that; libcanard/o1heap are not internally locked.
- Node init is **lazy inside the first Run()** (`uavcan_main.cpp:680`), not in start()/ctor; CAN bring-up + IRQ-callback registration happen on the work-queue thread. The IRQ trampoline runs in CAN-IRQ context — it must stay trivial (just `ScheduleNow()` on the singleton). Two independent sources both call `Run()` (3 ms tick AND IRQ `ScheduleNow()`).

**Typed-layer / codegen (AP-2):**
- Trait fn-ptrs are `static constexpr` **members** (`cxx_wrappers.h:36-37`) — calls go through indirection and won't inline (part of the bloat story; a PX4 trait should prefer plain static functions).
- `encode()` signature is **conditionally compiled**: `+bool !canfd` with `CANARD_ENABLE_CANFD`, `+bool tao` with `CANARD_ENABLE_TAO_OPTION`, else no extra arg (`cxx_wrappers.h:31-53`, `publisher.h:110-116`). The macro and every call site are `#ifdef`-forked — **PX4 must pick a TAO/CANFD policy up front** or arity won't match. (DroneCAN-v0 is classic-CAN, 8-byte — the FD branches are dead.)
- Bucket count is fixed at 8 (`CANARD_NUM_RX_BUCKETS`, `handler_list.h:38`), index `msgid % 8` (the data_type_id, NOT a hash); collisions walk linearly. `CANARD_NUM_HANDLERS` defaults 3 (`handler_list.h:31`) = max simultaneous interfaces; `head[][]` sized by it. `head[][]` + TransferObject/semaphore arrays are static globals that MUST be defined exactly once via `DEFINE_HANDLER_LIST_HEADS()`/`DEFINE_TRANSFER_OBJECT_HEADS()` (`handler_list.h:155`, `transfer_object.h:103`) — forgetting = link error, twice = ODR violation. (PX4 reimplements per-instance, sidestepping this.)
- Broadcasts deliver to ALL matching subscribers in the bucket; non-broadcast dispatch stops after the first successful handler (`handler_list.h:95-99`). So multiple PX4 modules can subscribe the same broadcast msgid and all receive it.
- `Subscriber<T>` only registers `CanardTransferTypeBroadcast` (`subscriber.h:42`); services go through `Server<T>` (Request) / `Client<T>` (Response) — don't subscribe a service type. `Client<T>` holds a SINGLE in-flight request, correlating strictly on `server_node_id` AND `transfer_id` (`service_client.h:61-62`); concurrent requests on one Client mis-correlate.
- The trait is emitted only when BOTH `__cplusplus` AND `DRONECAN_CXX_WRAPPERS` are defined (`msg.h.em:38,304`); generated C structs/codecs are fully usable WITHOUT the wrappers — a lighter PX4 shim can bypass the AP template layer entirely. Generated headers are **flat-named by full dotted DSDL name** (e.g. `include/uavcan.equipment.air_data.StaticPressure.h`), aggregated into `dronecan_msgs.h` (`dronecan_dsdlc.py:235`) — the dotted filename must be on the compiler include path.

**Cyphal-side reuse traps (CY-1/CY-2):**
- The generic `uORB_over_UAVCAN_Subscriber<T>` does a **RAW `reinterpret_cast`** of the CAN payload to `T*` and only checks `sizeof(T)` (`uorb_subscriber.hpp:78`) — it assumes the uORB struct IS the wire format (a PX4↔PX4 "uorb." transport). This is **NOT valid for real DroneCAN-v0 messages** (bit-packed, layout ≠ C struct). For v0 you MUST go through generated `*_decode`/`*_encode`.
- `user_reference` is the linchpin: set to the owning subscriber in the ctor (`BaseSubscriber.hpp:60`), echoed back by `canardRxAccept` (`CanardHandle.cpp:137`). A `CanardRxSubscription` created without `user_reference` null-derefs at `callback()`. (v0 has the same field/contract.)
- Cyphal's O1Heap allocator is a **file-scope global** (`cyphal_allocator`, `CanardHandle.cpp:57`) → exactly ONE `CanardHandle` can exist; the dtor frees the memalign'd arena with `delete` (mismatched alloc/free, `CanardHandle.cpp:94`) and leaves a dangling global. Both backends fix the interface to `can0`/`/dev/can0` (single bus, no parameterization); `canardRxAccept` is hardcoded `redundant_transport_index=0` (`CanardHandle.cpp:122`). The two backends have **inconsistent payload-ownership**: SocketCAN repoints the payload at its internal buffer (`CanardSocketCAN.cpp:194/200`, FIXME), NuttXCDev memcpys into the caller buffer (`CanardNuttXCDev.cpp:145`). CAN FD is effectively disabled in SocketCAN (`bool can_fd = 0` HOTFIX, `CanardSocketCAN.cpp:58-59`). A ported media layer must pin down one frame-ownership contract and one bus-parameterization story.
- Cyphal codegen runs at **configure time** (`execute_process`, won't re-run on DSDL edits without re-configure; failure aborts configure) — prefer the v0 build-time `add_custom_command`. The `CONFIG_CYPHAL_*` toggles reach code three ways (Kconfig + CMake `-D` foreach + `#ifndef` fallbacks) and size the binder array via preprocessor — a missing toggle silently shrinks it.
- Battery.hpp only publishes from the SourceTs branch (`:122`); Status/Parameters branches merely mutate the shared member (a latching/aliasing pattern, arguably buggy — don't copy literally). The whole Run() + both WorkItems are serialized by a single `_node_mutex` — callbacks may assume single-threaded canard access.

**Feature-parity traps (UA-1/2/3):**
- Pool block size is **48 via `-DUAVCAN_MEM_POOL_BLOCK_SIZE=48`** (`uavcan/CMakeLists.txt:84`), not the header default 56 — the new v0 pool is sized differently (per-frame nodes).
- Two SEPARATE remote-param paths must not be conflated: (a) the async MAVLink bridge FSM inside `Run()` (non-blocking, `_param_getset_client` + dirty bitmap), and (b) the blocking nuttx-CLI helpers (transient ServiceClients + `usleep` busy-waits). `Run()` guards the async path with `!_param_list_in_progress && !_param_in_progress && !_count_in_progress`. CLI `set_param` implicitly commits (calls `save_params()`, `:377`) unlike the MAVLink set path (marks dirty, waits for `PREFLIGHT_STORAGE`).
- v0 `ServiceCallResult` carries `getCallID().server_node_id` for response↔node correlation; **libcanard gives no automatic binding** — track `(node_id, transfer_id, service)` yourself. The bridge is strictly single-flight (v0 ServiceClient queues exactly one call) — a libcanard port must replicate the serialization or clobber outstanding transfers.
- On save/erase failure the dirty bit is **intentionally LEFT SET** (`uavcan_main.cpp:1397-1408`) for retry — don't "fix". `param_count` tolerates `-ErrInvalidParam` as "no node on bus" (`:1340`) — preserve so absent nodes don't spam. `response.param_id` hard-truncated to 16 chars (`:1304`) to match MAVLink width. `RestartNode` requires `magic_number = MAGIC_NUMBER` (`:1385`).
- DNA `StorageCANDNA` is only 1024 B, MAX_NODE_ID=125 (`static_assert` `AP_DroneCAN_DNA_Server.cpp:49`); AP stores a 6-byte FOLDED FNV-1a hash, NOT the raw 16-byte UID, so two UIDs can theoretically collide; an all-zero hash is indistinguishable from "empty" (`:213-224`). Allocation FSM resets `rcvd_unique_id_offset` on follow-up timeout AND on **every** first_part message (`:517-522`) — missing this deadlocks multiple simultaneous requesters. Allocation responses **must never be CAN FD** (`broadcast(rsp, false)`, `:559`). The DNA Database is a single static instance shared across buses, every method holds a semaphore — PX4 must decide single-shared vs per-bus and guard accordingly. StorageCANDNA offset differs by board (14336 vs 15232, `StorageManager.cpp:88/95`).
- `handleNodeStatus`/`handleNodeInfo` reject `source_node_id==0` or `>MAX_NODE_ID`; `handle_allocation` conversely REQUIRES `source_node_id==0` — easy to invert. `verify_nodes` excludes own `self_node_id` (`:303-304/319`); the monitor must exclude its own node ID.
- GPS backend resolution `get_dronecan_backend` (`AP_GPS_DroneCAN.cpp:243`) has side effects — auto-registers unknown `(driver,node_id)` AND re-sorts the table on every miss. A naive PX4 port must not assume the node-ID→instance mapping is stable/idempotent. (PX4 publishes uORB multi-instance keyed by source node-ID instead.)
- **ESC failure parsing is migration-introduced coupling**: `get_failures` depends on `dronecan_node_status` (from `NodeStatusMonitor`) AND `device_information` uORB; if the new driver drops that node-status→uORB pipeline, ESC failure flags silently read HEALTH_OK. ESC `StatusExtended` callback does NOT publish — it mutates the cached `esc_report` and relies on a following `Status` frame; a node sending only Extended never reaches uORB. ESC rate gate uses `_node.getMonotonicTime()` (1000000/400 = 2500 µs) — the port must substitute an hrt gate AND there is a second cap via `MixingOutput::setMaxTopicUpdateRate(1e6/400)` (`uavcan_main.cpp:123`).
- ESC RawCommand and Servo ArrayCommand carry **NO disarmed value** for ESC (UAVCAN_EC has no disarmed param, `module.yaml:87-90`) — "stopped" is whatever the mixer outputs; the controller never injects a stop frame. UAVCAN_SV does have disarmed=500 (→ 0.0 after /500-1). ArmingStatus treats an in-progress actuator test as FULLY_ARMED (test flag pushed from the ESC WorkItem, `:973` — compiled out without `CONFIG_UAVCAN_OUTPUTS_CONTROLLER`).
- Aux gating is layered: compile-time `CONFIG_UAVCAN_*` (`Kconfig:9-35`) AND runtime (ArmingStatus `UAVCAN_PUB_ARM==1`, ESC `UAVCAN_ENABLE>2`, Beep `CBRK_BUZZER`, RGBLED `UAVCAN_LGT_NUM>0`); LogMessage has no gate. Three DSDL vendor namespaces in use: **ardupilot** (SafetyState, MovingBaselineData), **dronecan** (RemoteID), **uavcan** — all must be generated for libcanard or those controllers won't compile. RGBLED light params cached at init (reboot to change); `MAX_NUM_UAVCAN_LIGHTS=2` must match `module.yaml`.
- SD FW serving hard limits: max 10 `.bin` per scan (`uavcan_servers.cpp:192`), `UAVCAN_MAX_PATH_LENGTH=168`, `FW_DB_LINE_SIZE=256`; migrate uses a shared static 512 B `_buffer` (`:62`) — **not reentrant**. `device_information` array grows by realloc-one (`node_info.cpp:245`); a single node id may hold MULTIPLE rows (one per capability); `registerDevice` merge logic (`:81-158`) is subtle — reproduce its dedup/merge exactly. `handleNodeInfoUnavailable` is a no-op (`:64`): a node failing GetNodeInfo never produces a `device_information` row. `_check_fw` forces `invalidateAll()` (`:749-752`) — keep an equivalent.
- **Don't replicate these bugs** (reproduce the BEHAVIOR/CLI, not the bug): `reset_node()` checks `if (!call_res)` treating success(0) as failure (`uavcan_main.cpp:248`); the status loop off-by-one `if (i > UAVCAN_NUM_IFACES) break;` (`:1003`, should be `>=`); the stale hpp comment block (`:192-201`) claims 32-byte buffers — trust the code constants (`ScheduleIntervalMs=3`). Dead/separate code: `cb_setget` (`:288`) + `cb_count` decl belong to the CLI path, NOT the MAVLink `cb_getset`/`param_count` bridge — don't conflate. RemoteID has a harmless duplicated `altitude_geodetic=-1000` (`remoteid.cpp:110-111`).

---

## 8. Bearing on the §8 open questions

One subsection per §8 open question that a slice touched, with the concrete recommendation the evidence supports. (Reader-internal numbering is reconciled to the briefing's canonical §8 list.)

### Q1 — Typed codec layer (highest-leverage; AP-2 + CY-2 are the direct evidence)

**Recommendation: keep the trait SEAM, drop the templates and fn-ptr members.** AP's answer is a per-message `cxx_iface` trait (`static constexpr ID/SIGNATURE/MAX_SIZE` + encode/decode **function-pointer members**) consumed by ~140-line header-only `Publisher/Subscriber/Server/Client` templates. The veneer is thin in *logic* (encode→send, decode→callback) but costs exactly the libuavcan-class bloat the migration targets: (a) one template instantiation incl. a stack buffer sized to `MAX_SIZE` **per message type**, (b) virtual `Callback<T>` dispatch, (c) malloc-based dynamic subscription + tid map, (d) fn-ptr indirection that blocks inlining. Cyphal independently proves the codec is a thin, swappable seam — RX = `canardRxAccept` then generated `*_deserialize_` inside `callback()`; TX = generated `*_serialize_` then `TxPush` — and that the decode→orb-publish / orb-subscribe→encode shapes are codec-agnostic (only the generated symbol set changes). **Design for P3:** define a `DronecanHandle` with `receive()/transmit()/TxPush/RxSubscribe` mirroring cyphal's, let `dronecan_dsdlc` supply `*_encode`/`*_decode`, and build a **non-templated descriptor-driven dispatcher** that takes `(id, signature, max_size, encode_fn, decode_fn)` from the trait (struct of static constexpr values + direct calls, not fn-ptr members), with at most a paper-thin typed inline wrapper. **Measure 2-3 types early** to confirm the typed layer stays flat (PLAN.md P3). Critical correctness constraint inherited either way: **`<type>_decode` returns TRUE on FAILURE.** Codegen *invocation* is itself part of this seam — wire `dronecan_dsdlc` at **build** time (`add_custom_command`) like the v0 driver, not at configure time like cyphal.

### Q2 — Scope / phasing (CY-2 / UA-3)

Already baked into PLAN.md P4 (GNSS + battery + ESC + DNA + SD-FW vertical slice). The evidence confirms the slice is coherent: the bridge pattern + Run()/scheduling + two-WorkItem structure are all reuse-as-is (mirror the cyphal shape), so a thin vertical slice exercises every reusable seam at once. The dynamic-port-ID/register layer and rcS exclusivity are reference-only/port-mapping (v0 is fixed-ID and **replaces** the `UAVCAN_ENABLE`/`DC_ENABLE` module, not an added branch).

### Q3 — DNA server source (AP-3 + UA-2)

**Recommendation: re-host PX4's `UavcanServers` behavior on libcanard, using AP's `AP_DroneCAN_DNA_Server` as the algorithmic reference.** The PX4 node's **own** node id is a static `UAVCAN_NODE_ID`/`DC_NODE_ID` param, validated `isUnicast()` (`uavcan_main.cpp:682-688`) — PX4 is **not** a DNA client; the `CentralizedServer` (gated `UAVCAN_ENABLE>1`) only allocates ids to OTHER nodes and persists to `/fs/microsd/uavcan.db`. So "DNA source" for self = param; for peers = the optional centralized server. Port-mapping disposition: reproduce the two-step UID allocation FSM (AP `handle_allocation`: ignore non-zero src, reset on follow-up timeout AND every first_part, assign down from 125, never CAN-FD the Allocation msg) + persistent node-ID↔UID table + the 5 s `verify_nodes` duplicate/dropout monitor. The v0 `FileStorageBackend`/`FileEventTracer` **file format** is libuavcan-specific → keep the persistence *behavior*, not the format (PX4 may store raw 16-byte UID rather than AP's space-saving 6-byte fold). Decide single-shared vs per-bus DB and guard accordingly.

### Q4 — MAVLink remote-param bridge & RemoteID sequencing (UA-2 + UA-3)

Both confirmed for **P6** (deferred after the core slice). The MAVLink bridge is a single-flight FSM pumped from `Run()` (`uavcan_main.cpp:771-970`) with four mutually-exclusive in-progress flags; its correctness hinges on v0 `ServiceCallResult::getCallID().server_node_id`, which **libcanard lacks** — the port must replicate the serialization AND add manual `transfer-id↔node-id` correlation. Save/erase is a chained `ExecuteOpcode→RestartNode` sweep across the dirty bitmap (`cb_opcode`/`cb_restart`). RemoteID is the heaviest aux controller (1 Hz, 5 broadcasts + ArmStatus rx, `dronecan` vendor DSDL, pulls `open_drone_id_translations.hpp`) — defer until core/sensor slices land. Aux controllers generally are libuavcan `TimerEventForwarder` periodic callbacks today; the new driver needs an equivalent periodic-work mechanism, independent of the MixingOutput WorkItems and gated by a mix of compile-time `CONFIG_*` + runtime params.

### Q5 — Memory model (CY-1 + AP-1 + UA-1 are the concrete evidence)

**Recommendation: drop O1Heap and the libuavcan `HeapBasedPoolAllocator` entirely; give the v0 driver a statically-sized internal pool buffer.** This is the cleanest-cut decision in the set. v1/cyphal = external O1Heap arena (8 KB, `CanardHandle.cpp:65-66`) + alloc/free callbacks + app-side `memory_free` of every RX payload (`:145`) and TX frame (`:173`), allocation can fail under load (Robson bound). Legacy v0 = a **fixed internal pool** handed to `canardInit` with NO app-visible alloc/free and NO O1Heap dependency, sliced into 32-byte blocks (40 w/ deadline, 128 w/ CAN-FD) serving BOTH RX reassembly states and TX queue items from one free-list. libuavcan today = `HeapBasedPoolAllocator` block 48, soft 250 / hard 500, IRQ-critical-section-synced, malloc-backed, grows on demand; `shrink` frees unused reserve, the dtor warns on leaks. **For P3:** PX4 owns a word-aligned byte arena, passes a sized region via a POOL byte-size param mirroring AP's (default 8192 non-FD / 16384 FD); surface `peak_usage_blocks` via `canardGetPoolAllocatorStatistics`. The 8 KB cyphal number and the 250/500×48 B libuavcan budget do **not** transfer 1:1 (v0 pools are sized in fixed-size frame nodes) — use them only to bound the budget. **`dronecan shrink`** likely becomes a no-op/alias (a fixed pre-allocated pool has nothing to free back). Allocator threading follows directly from the RX model: all-in-one-WorkItem ⇒ no internal locking, `CANARD_ALLOCATE_SEM=0`; interrupt/threaded RX intake ⇒ wire the `CANARD_ALLOCATE_SEM` hooks or external locks around the shared pool. Because RX/TX contend for the **same** pool, size it for worst-case simultaneous reassembly + TX backlog.

### Q6 — Param-migration details (UA-1 + UA-2)

Touched indirectly. `UAVCAN_*`→`DC_*` is decision 4; the readers confirm the semantics to preserve: ENABLE 0/1/2/3 tiers (all three thresholds — rcS start gate >0, `UavcanServers` >1, outputs >2), NODE_ID 1-125, BITRATE, ESC_IFACE bitmask, SUB_*/PUB_* toggles, EC*/SV* actuator groups. The 16-char name limit makes `DC_` strictly roomier. rcS keying mirrors with `DC_ENABLE` + the new binary name, preserving the mutual-exclusion-with-cyphal else-branch. (Exact name map + run-timing/idempotency are P3/P6 design work; no new constraints surfaced beyond the gating thresholds.)

### Q7 — Validation, ESC latency (AP-1 + UA-3)

The output path has a **double rate cap**: the controller-internal 400 Hz gate (`esc.cpp:93`, currently libuavcan monotonic clock → substitute hrt, 1e6/400 = 2500 µs) PLUS `MixingOutput::setMaxTopicUpdateRate(1e6/400)` (`uavcan_main.cpp:123`). For latency, AP's lever is the **immediate synchronous flush** — `processTx(true)` filtered to ESC RawCommand IDs, fired right after the ESC broadcast from the actuator publish path (`AP_DroneCAN.cpp:860`), so commands hit the wire the same call instead of waiting for the next tick; AP also sets `esc_raw` priority HIGH-1 / 2 ms (one above act_out_array's HIGH/5 ms) to win arbitration. PX4 uses RawCommand priority `NumericallyMin` (highest) + the `UAVCAN_ESC_IFACE` bitmask (`esc.cpp:79`). **For P3/P7:** map AP's immediate-flush onto a `DronecanHandle::flush()` callable from the ESC publish path (preserve the ESC-only filter idea — the fast-path filter must include every msg type intended for it, or those frames wait for the next normal drain) and validate ESC output latency (immediate-flush vs WQ tick) per PLAN.md P7, alongside `size -A` before/after on `ark_fmu-v6x` to confirm the ~64 KB reclaim.
