# DroneCAN libcanard driver — DESIGN (P3 lock candidate)

Engineer-facing design for `src/drivers/dronecan/`, the libcanard DroneCAN-v0 driver
replacing the libuavcan `uavcan` module (~124 KB → ~53 KB; **~64 KB reclaimed** on
`ark_fmu-v6x`, 99.9% → ~96.6%). Synthesized from the P3 judge panel. All `path:line`
citations are from `dronecan_migration/REFERENCE_MAP.md` (REF) unless prefixed `AP`
(ArduPilot tree) or `ACCEPTANCE_SPEC` (the field-map oracle).

PX4 root = `/home/jake/code/jake/PX4-Autopilot`.

---

## 1. Chosen architecture + why

**Backbone: MAINTAINABILITY-FIRST.** A strict, shallow layer stack
(media → transport → dispatch → codec-shim → bridges/services) where the two
easiest-to-forget legacy-libcanard contracts — `decode` returns **TRUE on failure**
and `encode` returns **byte length** (REF:238) — are interpreted in **exactly two
places**, and the whole stack is drivable off-target from a fake media interface. This
backbone won the aggregate because it is simultaneously the flattest codec layer
(judges: flat in message count, not just small) AND the lowest silent-corruption-risk
shape: bridge authors physically cannot open-code the decode negation.

### Aggregate scores

| Draft | Flash/Q1 | PX4-correctness/risk | Maintainability/feasibility | **Aggregate** | Killers |
|---|--:|--:|--:|--:|---|
| **maintainability-first** (backbone) | 9 | 9 | 9 | **27** | none |
| ardupilot-faithful | 9 | 9 | 8 | 26 | none |
| cyphal-symmetric | 8 | 9 | 8 | 25 | none |
| flash-minimal | 9 | 8 | 7 | 24 | none |

No draft carried an unresolved killer; the px4-correctness lens cleared all four. The
two design deltas the lenses disagreed on (pool block size; runtime-vs-static pool
array) are resolved decisively in §3/§9 below against the AP source, not carried.

### Grafts onto the backbone

- **From ardupilot-faithful (the behavioral oracle):** every libcanard touch point is a
  near-line-for-line re-expression of a cited AP source location, so the protocol risk
  surface equals AP's (field-proven). Specifically grafted: the `processTx(raw_commands_only)`
  ESC-only flush **filter** (the backbone dropped it as a full drain — restored, see §6);
  the DNA `handle_allocation` FSM ported near-verbatim (§5); the TAO/CANFD/DEADLINE
  compile-flag policy taken from AP's real build (boards.py:547,1325 — see §4/§9);
  the "keep persistence behavior not file format, store raw 16-byte UID" DNA decision (§5).
- **From cyphal-symmetric (the integration oracle):** the explicit "shell vs guts"
  boundary table drawn exactly where REF §4.1 draws it (the 5 libcanard-touching method
  bodies), reviewable line-by-line against `CanardHandle.cpp`; the verbatim reuse of the
  media layer, `Run()` `transmit();receive();transmit();` double-flush, two-WorkItem
  MixingOutput split, and `wq:uavcan` scheduling.
- **From flash-minimal (the floor):** generate **without** `DRONECAN_CXX_WRAPPERS` so the
  `cxx_iface` trait class is never even emitted (strictly better than shrinking it —
  removes the fn-ptr-member indirection at the source, REF:262/257); the
  hard rule of **exactly one** encode call site and **one** decode call site in the whole
  binary, enforcing both return-code contracts structurally; the Step-2 "measure 3 types
  BEFORE building any bridge" gate as a first-class go/no-go.

The backbone's own load-bearing decisions are kept: the single `DroneCANCodec` decode/encode
boundary, the per-instance `DronecanRxRouter` (sidesteps AP's static-global
`DEFINE_HANDLER_LIST_HEADS` ODR trap AND cyphal's silently-shrinking `UAVCAN_SUB_COUNT`),
the single named bucket constant that fails loudly on a missing toggle, and `DC_POOL_EXHAUSTED`
as an observable perf-counter event.

---

## 2. Module / file structure of `src/drivers/dronecan/`

Five layers, each one job and a testable boundary.

```
src/drivers/dronecan/
  dronecan/                      (submodule) legacy DroneCAN-v0 libcanard (canard.c/.h) — decision 1
  dronecan_dsdlc/                (submodule) AP DSDL-v0 C codegen — decision 1
  build/<board>/.../dsdlc_generated/   per-type struct + *_encode/*_decode + *_ID/_SIGNATURE/_MAX_SIZE/_NAME

  CanardInterface.hpp                  media base (init/close/transmit/receive)         [cyphal verbatim, re-typed]
  CanardSocketCAN.{hpp,cpp}            CONFIG_NET_CAN backend (SITL transport)          [cyphal verbatim, re-typed]
  CanardNuttXCDev.{hpp,cpp}            CONFIG_CAN backend                               [cyphal verbatim, re-typed]

  DronecanHandle.{hpp,cpp}            transport: cyphal CanardHandle SHELL, legacy v0 GUTS. Owns CanardInstance,
                                       the static pool, the DronecanRxRouter, the media iface. The ONLY PX4↔libcanard boundary.
  DronecanRxRouter.{hpp,cpp}          per-instance allocation-free dispatch registry: [id%N] buckets;
                                       accept() writes out-signature; dispatch() broadcast=all / service=first.
  DroneCANCodec.{hpp,cpp}             paper-thin shim: the ONLY caller of generated *_decode/*_encode.
                                       decodeTransfer() negates TRUE-on-failure; encodeBroadcast() checks len>0
                                       and builds the per-call stack CanardTxTransfer. Defines TypeDescriptor POD.
  RxSubscriberBase.hpp                one virtual handle(const CanardRxTransfer&) + the const TypeDescriptor list a bridge registers.
  TxPublisherBase.hpp                 persistent per-(dtid,type,dest) transfer_id + publish(struct) funnel through the shim.

  DronecanNode.{hpp,cpp}             DronecanNode : ScheduledWorkItem, ModuleParams; singleton on wq:uavcan.
                                       Lazy init in first Run(); 3 ms tick + CAN-RX-IRQ ScheduleNow(); transmit/receive/transmit
                                       under _node_mutex; owns the handle, the bridge/service registries, the two MixingOutput
                                       WorkItems; hosts the CLI + DC_* params + the migration call.

  bridges/ (Gnss Battery Baro Mag Accel Gyro Airspeed DiffPressure Flow Rangefinder
            Hygrometer FuelTank IceStatus SafetyButton GnssRelative).{hpp,cpp}
            + SensorBridgeBase (shared UavcanSensorBridgeBase channel-alloc/device-id mixin, REF:66-71)
  actuators/ (EscController ServoController HardpointController).{hpp,cpp}
            + MixingInterfaceEsc/Servo (OutputModuleInterface WorkItems, reuse-as-is, REF:199,227)
  services/  DnaServer FwServer NodeInfoRetriever NodeStatusMonitor NodeInfoPublisher
             TimeSync LogForward CanInterfaceStatus .{hpp,cpp}
  aux/       ArmingStatus SafetyState Beep RgbLed [P5]   RemoteId MavlinkParamBridge [P6]

  test/  FakeCanardInterface.hpp + dronecan_codec_test.cpp + dronecan_bridge_test.cpp + replay fixtures
  CMakeLists.txt  Kconfig  module.yaml  dronecan.params.yaml
```

**Bridge count note (a maintainability nit the panel flagged):** PLAN's "16 sensor
bridges" = 15 concrete `bridges/` classes + the `DC_SUB_BAT==2` *filtered-battery* mode
(a sub-path of the Battery bridge, not a separate file). The authoritative inventory is
the ACCEPTANCE_SPEC feature checklist; `bridges/` enumerates 15.

---

## 3. DronecanHandle — legacy API wrapping + the pool model (Q5)

`DronecanHandle` is the cyphal `CanardHandle` **shell** (`CanardHandle.hpp:40`: one class
owning the canard instance + media interface, exposing `receive()/transmit()/TxPush/
RxSubscribe-equivalent + node_id/mtu/diagnostics`, REF:94,215) with the five
libcanard-touching bodies rewritten to the legacy v0 C API (REF:96-106). This boundary is
the cyphal-symmetric graft — reviewable method-by-method against `CanardHandle.cpp`.

**Shell KEPT verbatim / guts REWRITTEN (the 5 methods):**

| Method | cyphal (modern) interior — REPLACE | legacy v0 interior — NEW |
|---|---|---|
| ctor/init | `o1heapInit`+`canardInit(&alloc,&free)`+`canardTxInit` (CanardHandle.cpp:57-86) | `canardInit(&_canard, _pool, pool_bytes, &onReception, &shouldAccept, this)` (canard.c:67, REF:17,34) |
| receive() | `canardRxAccept` pull + app-side `memory_free` (CanardHandle.cpp:111-152) | loop `_can_interface->receive(&f)` → `canardHandleRxFrame(&_canard,&f.frame,f.ts)` (canard.c:403); **push model, no app-side free** (REF:101) |
| transmit() | `canardTxPeek`/`canardTxPop` + free (CanardHandle.cpp:154-175) | `canardPeekTxQueue`/`canardPopTxQueue` drain (REF:22,102) |
| TxPush | `canardTxPush` | `canardBroadcastObj` (REF:19) / `TxRequestRespond`→`canardRequestOrRespondObj` (REF:20) |
| RxSubscribe | `canardRxSubscribe` per-port (CanardHandle.cpp:185) | **none** — v0 has no per-port subscribe (REF:104); handlers register with `_router`, acceptance decided in `shouldAccept` |

**Two static trampolines** (cast `ins->user_reference` → `DronecanHandle*`, AP_Canard_iface.cpp:177,182, REF:35-36):
- `shouldAccept(ins, out_sig, dtid, ttype, src)` → `_router.accept(dtid, ttype, out_sig)`. **MUST write `*out_sig`** from the matched descriptor or RX fails CRC (`canard.h:284`, REF:36,245). Kept cheap and side-effect-free — libcanard calls it a **second** time on mid-transfer frames with no RX state (`canard.c:465`, REF:245). A registry miss returns false (never falls through).
- `onReception(ins, transfer)` → `_router.dispatch(transfer)`. No free (push model frees internally).

**TX construction is STACK-LOCAL per call** (graft, REF:248): `TxPush`/`TxRequestRespond`
build the `CanardTxTransfer` on the stack — never AP's reused member transfer
(`AP_Canard_iface.h:83`), eliminating AP's `_sem_tx` shared-state hazard for free.
`canardBroadcastObj` returns the frame count or **negative on OOM**, handed back to the
codec shim. `canardRequestOrRespondObj` requires `node_id!=0` and auto-increments the tid
**only for Request**; responses reuse the request tid (`canard.c:379-382`, REF:246).
`canardCleanupStaleTransfers` runs once per pump pass (REF:23). `node_id` starts 0
(anonymous); `canardSetLocalNodeID` is called only after `DC_NODE_ID` validates `isUnicast`
(`uavcan_main.cpp:682-688`). **No TX is issued before `DC_NODE_ID` is validated & set** —
this closes the anonymous-node restriction (payload>7 rejected, request/respond fails;
`canard.c:200/365`, REF:247) by construction; PX4-as-server never TXes anonymously (DNA
*requesters* are the anonymous ones, and PX4 only receives those). AP do-not-copy extras
(`add_11bit_driver`, `write_aux_frame`, `CANARD_MULTI_IFACE`/`iface_mask`, the `while(true)`
`process()` loop + `sem_handle.wait`) are NOT ported (REF:118,170-171); only the
per-iteration body `processRx → processTx → cleanupStale` is taken.

### Pool model (Q5)

**Drop O1Heap AND the libuavcan `HeapBasedPoolAllocator` entirely.** The legacy library
owns exactly ONE arena: `canardInit` slices it into `pool_capacity = arena/CANARD_MEM_BLOCK_SIZE`
fixed blocks (`canard.c:95`, REF:17); **both** `createTxItem` (`canard.c:1338`),
`createRxState` (`canard.c:1513`), and `createBufferBlock` (multi-frame RX payload buffers)
`allocateBlock()` from the **same** free-list (REF:240). So RX reassembly and TX enqueue
contend; `canardBroadcastObj <= 0` may mean **OOM, not a full bus**.

- **Storage:** `DronecanHandle` owns a fixed, **word-aligned** `static uint32_t _pool_storage[DC_POOL_MAX/4]`
  (mirrors AP's `uint32_t[]` arena at `AP_DroneCAN.cpp:345`). The amount actually handed to
  `canardInit` is `min(DC_POOL_bytes, sizeof(_pool_storage))` — see §9 for why the param is a
  **bound on a max-sized static buffer**, not a runtime resize of the array (this resolves the
  px4-correctness "runtime param can't size a static array" inconsistency).
- **Block size = 40 B.** Built with `CANARD_ENABLE_DEADLINE=1` (we populate `.deadline_usec`
  for TX, matching AP boards.py:547 — the field is written unconditionally in
  `canardBroadcastObj`, AP_Canard_iface.cpp:93), so the block is 40 B not 32 (`canard.h:112-117`).
  `CANARD_ENABLE_CANFD=0` (classic 8-byte CAN), `CANARD_ALLOCATE_SEM=0`.
- **Default `DC_POOL = 8192` bytes** (AP's non-FD default, REF:240), exposed as an int32
  param. At 40 B that is ~204 blocks; `DC_POOL_MAX` (the static buffer) is sized larger
  (e.g. 16384) so a high-node-count bus can grow the live pool without a recompile. The
  cyphal 8 KB-O1Heap and the libuavcan 250/500×48 B numbers do **NOT** port 1:1 (different
  block model, REF:194,309) — they only bound the budget. **The §9 SPIKE must validate
  the block budget against worst-case simultaneous reassembly + TX backlog** before the
  default is locked.
- **Observability (backbone graft):** `TxPush` treats `<= 0` as a named `DC_POOL_EXHAUSTED`
  condition — rate-limited `PX4_ERR` + a `perf_counter`, **never silent**. The driver
  publishes `canardGetPoolAllocatorStatistics().peak_usage_blocks` into `can_interface_status`
  every 100 ms so headroom is observable in flight logs and `dronecan status`.
- **`dronecan shrink` → documented no-op alias** (a fixed pre-allocated pool has nothing to
  free back, REF:309).
- **Locking:** all canard access is on the WQ thread (the IRQ trampoline only `ScheduleNow()`),
  so `CANARD_ALLOCATE_SEM=0` and no internal locking (REF:253). The one cross-thread touch —
  the ESC immediate-flush from the MixingOutput WorkItem — takes `_node_mutex` around the
  `peek/pop` walk (§6). **Invariant:** if a future change moves RX intake into IRQ context,
  the unlocked pool becomes a data race and `CANARD_ALLOCATE_SEM`/explicit locking becomes
  mandatory (REF:253-254).

---

## 4. Codec & dispatch design (Q1) — concrete, with the flatness argument

Three pieces; the typed surface stays **flat regardless of message count**.

**(a) Generated layer — zero per-type C++.** `dronecan_dsdlc` runs with
`DRONECAN_CXX_WRAPPERS` **undefined**, so the `cxx_iface` trait at `msg.h.em:38/304` is
suppressed (REF:262) and only plain C is emitted: `<type>_encode(msg,buf,tao)` →byte len,
`<type>_decode(const CanardRxTransfer*, msg)` →TRUE-on-fail, and `_ID/_SIGNATURE/_MAX_SIZE/_NAME`
`#define`s. This is the flash-minimal graft: the per-type explosion is killed **at the
codegen source** (no template body, no fn-ptr member to indirect through, REF:257).

**(b) The single codec shim `DroneCANCodec.cpp` — exactly two free functions:**
- `bool decodeTransfer(const CanardRxTransfer&, void *out, decode_fn fn)` — the **ONE** place
  the TRUE-on-failure bool is inverted: `bool fail = fn(&t, out); return !fail;`. Every
  caller writes `if (codec::decode(...)) { use(struct) }` with **normal polarity** and
  **cannot forget** the negation (REF:238).
- `int32_t encodeBroadcast(DronecanHandle&, const TypeDescriptor&, const void *in, uint8_t *tid, encode_fn fn, priority)` —
  the **ONE** place `encode()==0` is treated as failure (length-return, REF:238) and the
  **ONE** place the stack-local `CanardTxTransfer` is built (REF:248). Returns the
  handle's `<=0` so the caller flags `DC_POOL_EXHAUSTED`.

**(c) `TypeDescriptor` POD** `{uint16_t data_type_id; uint64_t signature; uint16_t max_size; const char *name;}`
populated from the generated `#define`s — the "keep the trait SEAM as **data**, drop the
templates and fn-ptr members" realization (REF:172, §8 Q1:293). ~16 B/type of flat
**rodata**, not code. The `name` (rodata strings, the only element that grows O(type-count))
is diagnostic-only and `#ifdef`-gated to non-flight builds where flash is tightest.

The single bridge-facing template is an optional 6-line
`template<typename T> bool decode(const CanardRxTransfer&, T&)` that forwards to the
non-template `decodeTransfer` via a generated `T → {decode_fn, descriptor}` traits map
(a tiny generated mapping, NOT the suppressed `cxx_iface` — kept generated so it is not
hand-maintained boilerplate). Its per-`T` body is a cast + one call, **inlined to nothing**.

**Flatness argument (the whole thesis):** the only per-type **machine code** is the
generated `_encode`/`_decode` (the ~28 KB irreducible marshalling libcanard *also* has) plus
one thin decode-then-publish function per bridge. There is **no** per-type stack buffer (the
decode target is a compiler-sized named C-struct **local in the bridge**, exactly as cyphal
ships at `Battery.hpp:102` / `Gnss.hpp:73`), **no** virtual `Callback<T>` hierarchy, **no**
fn-ptr trait member, **no** malloc'd tid map, and dispatch is one shared C router. Contrast
AP's `Publisher<T>`/`Subscriber<T>`, each materializing `msg_buf[MAX_SIZE]` + an
encode/decode body + a vtable per instantiation (REF:173) — the libuavcan-class explosion
being escaped. The accepted clarity spend is ~2-4 KB (two out-of-line codec functions +
descriptor rodata + `DC_POOL_EXHAUSTED` logging + ~18 per-bridge vtables, O(bridges) not
O(types)); §9 measures it.

**Dispatch — `DronecanRxRouter` (per-instance, allocation-free).** Reimplements AP's
allocation-free `[data_type_id % N]` bucketed `HandlerList` BEHAVIOR (broadcast delivers to
**all** matching handlers; service stops after **first**; `handler_list.h:95-99`, REF:174,260)
but **per-instance**, NOT via AP's static-global `head[][]` + `DEFINE_HANDLER_LIST_HEADS`
macros (which ODR/link-error if defined zero or twice, REF:174,259). `_buckets[DC_RX_BUCKETS]`
where `DC_RX_BUCKETS` is a **single named compile-time constant** from the enabled-bridge
set — NOT cyphal's three-file preprocessor-summed `UAVCAN_SUB_COUNT` that silently shrinks
on a missing toggle (REF:83,268). An `RxHandler` is `{dtid, ttype, signature, RxSubscriberBase*, next}`.
`RxSubscriberBase` carries **exactly one virtual** `handle(const CanardRxTransfer&)` — one
vtable per **bridge class** (~18 total), explicitly **not** per data-type; AP's `Callback<T>`
per-type virtual hierarchy + `allocate_sub_*` heap helpers are rejected (REF:175).
Registration is static (each bridge ctor calls `_router.add(this, descriptor)`; no
malloc-per-subscription). `accept()` and `dispatch()` walk the same bucket — one source of
truth, so the signature `accept()` writes always matches the handler that decodes (closes the
"accepted but wrong signature → CRC fail" class, REF:245). TX transfer-ids are per-publisher
`static uint8_t` keyed to `(dtid, ttype, dest)`, never shared, never stack (the lib mutates
them, `canard.c:224/381`, REF:246).

The cyphal `PublicationManager`/`SubscriptionManager` dynamic-port-ID/register layer
**collapses** to a flat compile-time list because v0 uses **fixed** data-type IDs
(`UavcanPublisher`/`DynamicPortSubscriber updateParam` is do-not-port, REF:224).

---

## 5. DNA + SD-card FW server (Q3)

**Re-host PX4's `UavcanServers` BEHAVIOR on libcanard, with AP's `AP_DroneCAN_DNA_Server` as
the algorithmic reference** (REF:301; ardupilot-faithful graft = near-line-for-line FSM).
PX4's **own** id is the static `DC_NODE_ID` param validated `isUnicast` — **PX4 is the
server, not a DNA client** (REF:135,301). The centralized server (gated `DC_ENABLE>1`) only
allocates ids to **other** nodes.

**DNA FSM** (`DnaServer`, an `RxSubscriberBase` on `dynamic_node_id.Allocation`; port of AP
`handle_allocation`, `AP_DroneCAN_DNA_Server.cpp:506`, REF:177) — each step is an easy
inversion, all enumerated:
- **REQUIRE `source_node_id == 0`** (the inverse of NodeStatus/NodeInfo which **reject** 0, REF:277).
- **Reset `rcvd_unique_id_offset` on follow-up timeout AND on EVERY `first_part_of_unique_id`**
  message (missing the latter **deadlocks** simultaneous requesters, REF:276).
- Accumulate UID chunks; assign the first free id scanning **DOWN from 125**; re-broadcast the
  (partial-or-complete) Allocation reply.
- **NEVER CAN-FD the Allocation response** (`broadcast(rsp,false)`, REF:177,276 — and CANFD is
  compiled out anyway).
- Do **NOT** fold node-id into the UID (an AP wart, REF:177).

**Persistence:** a node-ID-indexed table + an in-RAM `Bitmask<128>` presence map with
magic-validation + reset-on-mismatch (REF:178). Keep the persistence **behavior**, NOT the
libuavcan `FileStorageBackend` **file format** (REF:180); write to `/fs/microsd` (DC-named
file). **Store the RAW 16-byte UID** (PX4 SD is roomy — drop AP's 6-byte FNV-1a fold and its
theoretical collisions / all-zero-ambiguity, REF:178,276). Single shared DB (single bus),
guarded by `_node_mutex`. Allocation responses flush via `Run()`'s trailing `transmit()`.

**Node monitor:** 5 s `verify_nodes` sweep + reactive `GetNodeInfo` on new NodeStatus +
duplicate/dropout detection, **excluding own `self_node_id`** (REF:179,277) — a
safety-relevant prearm signal.

**SD-card FW server** (`FwServer`, decision 5, **in P4 scope**, REF:136,181): the POSIX
`FW.db`/`copy`/`migrate`/`validate` helpers port **nearly verbatim** (`uavcan_servers.cpp:245-407`);
only the file-server **transport** (`uavcan.protocol.file.Read` service responses, Server-role
descriptor) and the **version trigger** (`FirmwareUpdateTrigger` + `FirmwareVersionChecker` →
`BeginFirmwareUpdate`, Client-role) are rebuilt on libcanard service transfers via the same
`DronecanRxRouter`. Honor the hard limits: **max 10 `.bin`/scan**, `UAVCAN_MAX_PATH_LENGTH=168`,
`FW_DB_LINE_SIZE=256`; the shared static **512 B `_buffer` is NOT reentrant** (`:62`) — keep it
confined to the single WQ thread (REF:282). `NodeInfoRetriever` is hand-rolled (no libcanard
equivalent, REF:182): a `GetNodeInfo` client with `invalidateAll()`/`isNodeKnown()` + a
listener fan-out feeding `NodeInfoPublisher` (→`device_information`) and the FW trigger;
`_check_fw` forces `invalidateAll()` (REF:282). `NodeInfoPublisher` reproduces the
realloc-one growth where one node id may hold **multiple** `device_information` rows + the
subtle `registerDevice` dedup/merge exactly; `handleNodeInfoUnavailable` is a no-op (REF:282).

---

## 6. Scheduling / threading + ESC immediate-flush (Q7)

The exact UavcanNode/Cyphal work-queue model, **NOT** AP's thread (REF:118,250-254).

- `DronecanNode : ScheduledWorkItem, ModuleParams` on `wq:uavcan` (stack 3754, prio -19;
  may shrink once libuavcan templates are gone, **re-measure** — see §11).
- **Dual scheduling:** `ScheduleOnInterval(3 ms)` bounds TX latency / timeouts / stale-cleanup,
  PLUS a CAN-RX-IRQ `busevent_signal_trampoline` that does nothing but `ScheduleNow()` on the
  singleton (`uavcan_main.cpp:486-490`, REF:128,190,254). The IRQ trampoline stays trivial
  (CAN-IRQ context, no canard access).
- **Lazy init inside the first `Run()`** (`uavcan_main.cpp:680`, REF:254) — CAN bring-up + IRQ
  registration on the WQ thread, NOT in ctor/start.
- **`Run()` body** (cyphal `Cyphal.cpp:156-221`, REF:218): lock `_node_mutex` → lazy init → on
  `parameter_update` reconfigure → periodic node msgs (NodeStatus 1 Hz, time sync) →
  `_pub_manager.update()` → **`transmit(); receive(); transmit();`** → unlock. The
  double-transmit sandwich is **load-bearing**: `receive()` runs `canardHandleRxFrame` whose
  `onReception` callbacks enqueue service **responses** (DNA Allocation, GetNodeInfo, param
  GetSet); the trailing `transmit()` flushes them **the same tick** (REF:78,218).
  `canardCleanupStaleTransfers` once per pass.
- **Actuators = a second `OutputModuleInterface` WorkItem** (`MixingInterfaceEsc/Servo`) on the
  **same** `wq:uavcan`, so it serializes with `Run()` under the shared `_node_mutex`
  (REF:78,150,227) — no extra locking, `CANARD_ALLOCATE_SEM=0`. The two WorkItems can never be
  mid-pump simultaneously (one WQ thread pops sequentially); the mutex is correct and matches
  the cyphal shape. (The px4-correctness lens corrected a mis-statement here: the only true
  cross-context actor is the IRQ `ScheduleNow`, which touches no pool.)

**ESC immediate-flush (the AP latency lever, REF:117,169,279, AP_DroneCAN.cpp:860):** after the
ESC controller `encodeBroadcast`s `esc.RawCommand` from `updateOutputs()` in the MixingOutput
WorkItem, it immediately calls `DronecanHandle::flush()` == `processTx(raw_commands_only=true)`,
which drains **ONLY** frames whose `CANARD_MSG_TYPE_FROM_ID` matches
`UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID` (and `COM_HOBBYWING` if used), **walking past every
non-ESC queued frame** (`AP_Canard_iface.cpp:234-236`, REF:252). **This ESC-only filter is the
ardupilot-faithful graft** — the backbone proposed a full drain, which would also push
NodeStatus/service frames early and reorder non-ESC traffic; the filter is restored so the ESC
command hits the wire that call while non-ESC frames wait for the normal drain. `flush()` takes
`_node_mutex` (the queue/pool are not internally locked, REF:252-253). RawCommand priority
`NumericallyMin` (highest) + the `DC_ESC_IFACE` bitmask.

**ESC double rate cap preserved** (REF:279,316): a controller-internal hrt gate
`now - _prev < 1e6/400 = 2500 µs` (substituting libuavcan `getMonotonicTime`) **AND**
`MixingOutput::setMaxTopicUpdateRate(1e6/400)` (`uavcan_main.cpp:123`). ESC `StatusExtended`
**mutates the cache and does not publish** — it relies on a following `Status` frame (REF:279);
reproduce exactly. Aux controllers (ArmingStatus 10 Hz, Beep 100 Hz, …) get a small
periodic-tick dispatcher driven from `Run()` (replacing libuavcan `TimerEventForwarder`,
REF:305), independent of the MixingOutput WorkItems.

---

## 7. Param migration (Q4/Q6)

`UAVCAN_* → DC_*` (decision 4); `DC_` is strictly roomier under the 16-char limit (REF:313).
A `migrateLegacyParams()` driven by a static const `{ "UAVCAN_*", "DC_*" }` name-pair table.

- **Run timing / idempotency:** runs **once at module start, before any `DC_*` is read**,
  gated by a sentinel (`DC_CONFIG_VER`/`DC_MIGRATED`). For each pair: if `DC_x` is still at
  firmware default AND legacy `UAVCAN_x` exists and differs, copy/convert. Re-running is a
  no-op; a user who already set `DC_x` **wins** (non-destructive).
- **Semantics preserved exactly** (REF:196,313): `DC_ENABLE` with **all three thresholds**
  (>0 rcS start gate, >1 `DnaServer`+`FwServer`, >2 ESC/actuator outputs; REF:130,197),
  `DC_NODE_ID` 1-125 `isUnicast`, `DC_BITRATE`, `DC_ESC_IFACE` bitmask (default 255), the
  `DC_SUB_*`/`DC_PUB_*` per-bridge gates (pure compile+runtime gates since v0 is fixed-ID),
  the `DC_EC_*`/`DC_SV_*` actuator groups (per-channel FUNC/MIN/MAX/DIS/FAIL/REV). **Decided
  once (not left as a fork):** the user-facing actuator params are renamed `DC_EC_*`/`DC_SV_*`;
  the `MixingOutput` internal prefix strings are migrated to match so `module.yaml` and the
  controller strings stay consistent (no `UAVCAN_`/`DC_` split). `DC_POOL` (+ implicit
  `DC_POOL_MAX` build constant) has no `UAVCAN_` predecessor.
- **Legacy retirement:** keep `UAVCAN_*` **definitions** available for one transition release
  as the migration **source**; retire later. The new driver never reads `UAVCAN_*` except
  through the table. The old `uavcan` module is Kconfig-excluded anyway (decision 3), so no
  name collision.
- **rcS + the cutover chicken-and-egg (a real bug the px4-correctness lens caught):** rcS keys
  `if param greater DC_ENABLE 0; then dronecan start`, taking over the `UAVCAN_ENABLE` slot
  (no third branch, REF:84,230). But on a fresh board migrated from `UAVCAN_ENABLE>0`,
  `DC_ENABLE` is default 0 at first boot, so rcS would never start the module that performs the
  migration. **Resolution:** `migrateLegacyParams()` runs from a **board-init / param-default
  hook outside the gated module** (a small startup shim, like the existing param-default
  application) so `DC_ENABLE` is populated from `UAVCAN_ENABLE` **before** rcS evaluates the
  start gate. The MAVLink **remote-param bridge** (params of OTHER nodes over GetSet/
  ExecuteOpcode/RestartNode) is a **separate subsystem** (P6) — do not conflate it with this
  local `DC_*` migration (REF: two-separate-param-paths trap).

---

## 8. P4 vertical-slice implementation plan (ordered, buildable)

GNSS + battery + ESC + DNA + SD-FW behind `DRIVERS_DRONECAN`, then fan out. Each step has a
concrete acceptance criterion.

- **P4.0 Scaffold + codegen:** create `src/drivers/dronecan/`; vendor `dronecan/libcanard` +
  `dronecan_dsdlc` submodules; wire codegen via **build-time** `add_custom_command` + `.stamp`
  + `DEPENDS` (the `uavcan/CMakeLists.txt:121-130` form, NOT cyphal's configure-time
  `execute_process` which won't re-run on DSDL edits and aborts configure on failure, REF:82,268),
  `DRONECAN_CXX_WRAPPERS` **off**, generating **all three vendor namespaces** (ardupilot,
  dronecan, uavcan + com.hex + cuav) from the start (REF:281). Pin the compile flags on the
  libcanard target: `CANARD_ENABLE_TAO_OPTION=1`, `CANARD_ENABLE_CANFD=0`,
  `CANARD_ENABLE_DEADLINE=1`, `CANARD_ALLOCATE_SEM=0` (see §9 — fixes encode arity & block
  size up front). Kconfig `DRIVERS_DRONECAN depends on !DRIVERS_CYPHAL && !DRIVERS_UAVCAN` +
  CMake `message(WARNING)` (decision 2). **Acceptance:** empty module compiles/links on
  `ark_fmu-v6x` behind `DRIVERS_DRONECAN`; `dronecan_msgs.h` present; canard* symbols co-exist
  with neither uavcan nor cyphal.
- **P4.1 Media + transport + the two error boundaries:** copy `CanardInterface`/`CanardSocketCAN`/
  `CanardNuttXCDev`, re-type the frame struct to v0 `CanardCANFrame`, **pin the memcpy
  frame-ownership contract** (fix cyphal's SocketCAN-repoint vs NuttX-memcpy split, REF:267).
  Implement `DronecanHandle` (canardInit + 2 trampolines + static pool + receive/transmit/
  TxPush/`flush()` ESC filter + cleanupStale + `peak_usage_blocks`), `DronecanRxRouter`
  (accept/dispatch buckets), `DroneCANCodec` (the negating `decodeTransfer` + len-checking
  `encodeBroadcast`). **Refactor `DronecanHandle` to constructor-inject the media backend** so
  `FakeCanardInterface` can replace it (the cyphal ctor hard-`new`s the backend — that wiring
  must change). **Acceptance + gate:** `dronecan_codec_test` (encode→decode identity + a
  deliberately wrong-length transfer asserting `decode()==false`) and a fake-frame dispatch
  test pass **before any bridge** — this proves both traps are handled centrally.
- **P4.2 — MEASURE THE SEAM (the Q1 go/no-go, do BEFORE any bridge):** wire descriptors for
  **3 types** (`gnss.Fix2`, `power.BatteryInfo`, `esc.RawCommand`) and run
  `/dronecan-flash-delta` on `ark_fmu-v6x`. **Confirm per-type cost == generated `_encode`/
  `_decode` only, with NO template/vtable growth**, and that the optional `decode<T>` inline
  folds to ~0. If regressed, push more into the non-templated path before scaling.
- **P4.3 DronecanNode + lifecycle:** `ScheduledWorkItem` on `wq:uavcan`; lazy first-Run init;
  3 ms tick + CAN-RX-IRQ `ScheduleNow`; `transmit/receive/transmit` under `_node_mutex`;
  `canardSetLocalNodeID` from `DC_NODE_ID`; NodeStatus 1 Hz. CLI `dronecan {start|status|stop|
  shrink(no-op)|update|param}`; `DC_ENABLE/NODE_ID/BITRATE` + `migrateLegacyParams()` (via the
  out-of-module hook, §7); rcS keyed on `DC_ENABLE`. **Acceptance:** node boots, appears on the
  bus, `status` shows node id + `peak_usage_blocks`.
- **P4.4 GNSS bridge** (the `/dronecan-port-bridge` example): port ACCEPTANCE_SPEC §1 —
  Fix/Fix2/Auxiliary/RelPosHeading, the Fix2 mode→fix_type remap, the size-1/6/21/36 covariance
  unpack **with the intentional 21→36 FALLTHROUGH** (`gnss.cpp:217`, a latent bug — replicate
  byte-identically, do NOT "fix"), the microsecond leap-second UTC math, Auxiliary 2 s hdop/vdop
  fallback, Fix-vs-Fix2 per-node dedup, `make_uavcan_device_id` (devtype 0x85); RTCM/MBD uplink
  + `gps_dump`. Reuse `SensorBridgeBase` channel-alloc (4 channels by node_id). Each quirk a
  **named pure function** with a spec-cited unit test. Verify field-for-field; then SITL.
- **P4.5 Battery bridge:** port ACCEPTANCE_SPEC §9 — BatteryInfo/BatteryInfoAux/CBAT four
  decode paths with the exact precedence (Filter global+sticky; CBAT latches+blocks BatteryInfo;
  Aux upgrades Raw→RawAux), per-instance `_node_ids[3]` + `_battery_status[3]`/`_battery_info[3]`
  accumulators, K→C, CBAT current negation, the second `battery_info` topic. (Battery's ctor
  takes a `NodeInfoPublisher*` and calls `registerDeviceCapability`; it is nullptr-guarded —
  passes nullptr until P4.6, so `device_information` BATTERY rows are absent until then. Stated
  so a maintainer isn't surprised.)
- **P4.6 Node services (unblocks ESC failures):** `NodeStatusMonitor`→`dronecan_node_status`
  (NodeID↔uORB-multi-index map, exclude own id), `NodeInfoRetriever` (hand-rolled GetNodeInfo),
  `NodeInfoPublisher`→`device_information` (realloc-one + multi-row-per-node + dedup/merge),
  `CanInterfaceStatus` (`peak_usage_blocks` + bus counters). **These ship STRICTLY before the
  ESC slice is declared done** — `get_failures` reads `dronecan_node_status` + `device_information`
  and **silently reads HEALTH_OK** with no compile error if absent (REF:279).
- **P4.7 ESC actuator + immediate-flush:** `MixingInterfaceEsc` WorkItem (`DC_EC` prefix) +
  `EscController`. OUT RawCommand ≤400 Hz (hrt gate + `setMaxTopicUpdateRate`) via
  `encodeBroadcast`, priority `NumericallyMin`, `DC_ESC_IFACE` mask; `DronecanHandle::flush()`
  (ESC-only filter) immediately after broadcast. IN Status/StatusExtended →`esc_status`
  (esc_index addressing, 1200 ms freshness, the VertiQ/`iq_motion` failure parsing as a
  documented pure function). **Validate immediate-flush latency vs WQ tick**; add a regression
  test for a node sending **only StatusExtended**.
- **P4.8 DNA + SD-FW server:** `DnaServer` (two-step UID FSM, assign-down-from-125,
  never-CAN-FD, raw-16-byte-UID persistence to SD, 5 s `verify_nodes` excluding self) +
  `FwServer` (BasicFileServer file.Read + FirmwareUpdateTrigger + FirmwareVersionChecker + FW.db
  under `/fs/microsd/ufw`, POSIX helpers verbatim, transport rebuilt). Gated `DC_ENABLE>1`.
  Responses flush via the trailing `transmit()`. GlobalTimeSync master+slave and
  `debug.LogMessage`→`px4_log` (both small) land here.
- **P4.9 Flash gate + parity:** `/dronecan-flash-delta` on `ark_fmu-v6x_default`
  (`DRIVERS_UAVCAN` off, `DRIVERS_DRONECAN` on) — confirm the **~64 KB reclaim** nets inside the
  ~2-4 KB clarity spend; first lever if over: collapse descriptor tables into per-bridge
  `static const`, force-inline `encodeBroadcast` on the hot ESC path. Run `/dronecan-review` for
  spec-parity + decode-error + pool + DNA/FW parity on the GNSS/battery/ESC slice. **Then** fan
  out the 12 remaining sensor bridges + aux (P5) and MAVLink param bridge + RemoteID (P6).

---

## 9. What the Q1 measurement SPIKE must confirm before the design is locked

The whole ~64 KB thesis rides on Q1 flatness and the pool sizing. The P4.2 spike (3 types,
`/dronecan-flash-delta` on `ark_fmu-v6x`) must confirm, **before the design is locked**:

1. **Per-type cost is flat** — adding `gnss.Fix2` + `power.BatteryInfo` + `esc.RawCommand`
   grows `.text` by **only** the generated `_encode`/`_decode` bytes, with **no**
   template/vtable/per-type-`MAX_SIZE`-buffer growth. The `TypeDescriptor` adds only ~16 B/type
   rodata.
2. **The optional `decode<T>` inline folds to ~0** (verify with the flash-delta skill; if it
   doesn't, the non-templated `decodeTransfer` is already ergonomic enough — drop the wrapper).
3. **Generated dead-strip works** — generating all three namespaces but referencing only the
   enabled bridges' types must leave unused `_encode`/`_decode` unreferenced and **GC'd by
   `--gc-sections`** (per-function sections). Given ICF/LTO are exhausted/unsafe on this
   codebase (project memory), gc-sections is the only thing between "flat" and "all ~140 DSDL
   types ship". A 3-type harness won't surface a gc-sections failure on the full namespace —
   **re-confirm at the integrated P4.9 build**.
4. **Encode arity is unambiguous** under `CANARD_ENABLE_TAO_OPTION=1` + `CANARD_ENABLE_CANFD=0`
   — one `(struct*, buf, bool tao)` signature, `tao=true` at the call site; and **decode does
   not reject valid TAO-encoded multi-frame transfers** (with TAO on, decode relaxes the exact
   `byte_len != payload_len` check to `>`; REF:238). Validate against a **real peripheral**, not
   just the loopback fixture — this is the one decode-correctness gap a unit harness can miss.
5. **Pool block budget** — with 40 B blocks (DEADLINE=1) and `DC_POOL=8192` (~204 blocks),
   measure `peak_usage_blocks` under worst-case **simultaneous** multi-frame reassembly
   (GNSS Fix2 + battery) **plus** ESC/servo TX backlog. Confirm headroom and lock the default
   (or raise it / `DC_POOL_MAX`). This must pass before P4.9.

If 1-2 regress, the codec layer is reworked (push more into the non-templated path) before any
bridge is built. If 3-5 regress, sizing/flags are adjusted; the design lock waits on green.

---

## 10. Decision log — §8 question → locked resolution + source

| Q | Locked resolution | From |
|---|---|---|
| **Q1 codec** | Lighter PX4 shim, **95% non-templated**. Suppress `DRONECAN_CXX_WRAPPERS` (no `cxx_iface` trait emitted); one `DroneCANCodec` with two free functions as the **only** decode/encode call sites; `TypeDescriptor` POD as data; one optional inline `decode<T>` that folds away. Flat in message count. Measure 3 types at P4.2. | maintainability-first backbone + flash-minimal graft (wrappers-off); all 4 flash-q1 judges concur |
| **Q2 scope** | GNSS+battery+ESC+DNA+SD-FW slice behind `DRIVERS_DRONECAN`, with two re-orderings: **measure the seam (P4.2) before any bridge**, and **node-status/device_information (P4.6) before the ESC slice is done**. Then fan out. | all drafts; px4-correctness + maintainability judges |
| **Q3 DNA** | Re-host `UavcanServers` on libcanard, AP `AP_DroneCAN_DNA_Server` as algorithmic reference. Self id = static `DC_NODE_ID` (server, not client). Two-step UID FSM (require src==0, reset on first_part+timeout, assign down from 125, never CAN-FD), `Bitmask<128>` presence, **raw 16-byte UID** to `/fs/microsd` (behavior not format). 5 s `verify_nodes` excl. self. SD-FW: POSIX helpers verbatim, transport rebuilt. | ardupilot-faithful FSM fidelity + cyphal-symmetric persistence-behavior framing |
| **Q4 MAVLink/RemoteID** | Both **P6**. MAVLink bridge = single-flight FSM, manual `(node_id, transfer_id, service)` correlation (libcanard lacks `getCallID().server_node_id`), dirty-bit-set-on-failure, RestartNode MAGIC_NUMBER, param_id truncated to 16 (REF:274/275). RemoteID last (heaviest, vendor DSDL). Kept **separate** from the local `DC_*` migration. | all drafts; REF §7 traps |
| **Q5 memory** | Drop O1Heap + `HeapBasedPoolAllocator`. ONE word-aligned static pool to `canardInit`. **Block = 40 B (`CANARD_ENABLE_DEADLINE=1`, matching AP), CANFD=0, ALLOCATE_SEM=0.** `DC_POOL=8192` default = **bound on a max-sized static buffer** (`DC_POOL_MAX`), not a runtime array resize. RX/TX contend → `canardBroadcastObj<=0` is named `DC_POOL_EXHAUSTED` (perf-counter + log, never silent); `peak_usage_blocks` in `can_interface_status`. `dronecan shrink` = no-op alias. | maintainability-first (observability) + ardupilot-faithful (DEADLINE=1/40 B from AP boards.py:547) |
| **Q6 param migration** | Static `{UAVCAN_*,DC_*}` table; idempotent, sentinel-gated, non-destructive, **run from an out-of-module board-init hook before rcS evaluates `DC_ENABLE`** (resolves the cutover chicken-and-egg). Preserve all three ENABLE thresholds; rename EC/SV params **and** MixingOutput prefix strings consistently (decided, not forked). Keep legacy defs one release as the source. | maintainability-first + px4-correctness (cutover-bug fix; EC/SV fork resolution) |
| **Q7 validation** | Double rate cap (hrt 2500 µs gate + `setMaxTopicUpdateRate`) + ESC immediate `flush()` **filtered to ESC RawCommand IDs** (ardupilot-faithful graft over the backbone's full drain). First-class SITL/unit seam (`FakeCanardInterface` + recorded frames; codec round-trip + the decode-inversion contract test + field-for-field bridge tests). `/dronecan-flash-delta` before/after on `ark_fmu-v6x`. | ardupilot-faithful (filter) + maintainability-first (test seam) |

**Locked-decision compliance (all five, REF/PLAN §2.1):** (1) vendor `dronecan/libcanard` +
`dronecan_dsdlc`; (2) `DRIVERS_DRONECAN depends on !DRIVERS_CYPHAL && !DRIVERS_UAVCAN` + CMake
warning → resolves the `canard*` collision by never co-linking; (3) new `dronecan` module, one
Kconfig flip; (4) `UAVCAN_*→DC_*` idempotent migration; (5) DNA + SD-card FW server in P4 scope.

---

## 11. Top open risks for plan-mode review

1. **Q1 flatness + generated dead-strip are unproven until measured.** The whole ~64 KB rides
   on the typed layer staying flat AND `--gc-sections` stripping the unused codecs across the
   full 3-namespace generation (ICF/LTO are exhausted/unsafe here — gc-sections is the only
   lever). P4.2 measures 3 types; the dead-strip only shows at the integrated P4.9 build.
   *Mitigation:* hard P4.2 gate + re-confirm at P4.9; named fallback levers if over budget.
2. **Shared single pool starvation is observability-mitigated, not structurally prevented.** RX
   reassembly + TX (incl. multi-frame payload buffers) draw from one 40 B free-list; an
   undersized `DC_POOL` + noisy bus can drop **ESC** frames, and `canardBroadcastObj<=0` is
   ambiguous (OOM vs full bus). *Mitigation:* `DC_POOL_EXHAUSTED` telemetry + `peak_usage_blocks`
   + the §9 worst-case block-budget SPIKE + a recommended `DC_POOL` per board class at P7. No
   reserved-TX-headroom mechanism is proposed (AP ships the same single-pool model) — flag for
   review whether the safety-critical ESC path warrants reserved headroom.
3. **The single decode/encode boundary is a strength that is also a single point of
   correctness.** If `decodeTransfer`'s `return !fail` is ever inverted, **every** bridge
   silently processes garbage. *Mitigation:* the P4.1 contract test (corrupt transfer →
   `decode()==false`) lands before any bridge and is double-covered by per-bridge field-for-field
   tests — but the blast radius is one ~6-line function; confirm the test discipline is acceptable.

Secondary watch items for the gate: the **TAO decode-length** behavior must be validated against
a real peripheral, not just the loopback fixture (§9.4); the **`wq:uavcan` stack** adds deep new
call chains (DNA/FW POSIX file I/O, P6 MAVLink FSM) on the same WQ thread under `_node_mutex` —
a 512 B non-reentrant FW migrate buffer is the likeliest overrun culprit, so **re-measure
high-water after the slice** rather than assume it shrinks; and the **byte-identical quirk
replication** (Fix2 21→36 fallthrough, leap-second math, CBAT negation, battery precedence,
servo /500-1) has the field-for-field bridge tests as its only net — every spec row must be
covered or a missed quirk passes compilation invisibly.
