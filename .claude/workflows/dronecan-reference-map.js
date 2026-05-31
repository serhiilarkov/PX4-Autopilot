export const meta = {
  name: 'dronecan-reference-map',
  description: 'P1/Understand: fan out readers over the three DroneCAN reference codebases (ArduPilot AP_DroneCAN, PX4 cyphal, PX4 uavcan) and synthesize a consolidated REFERENCE_MAP.md for the libcanard migration design phase',
  whenToUse: 'Run before designing the new src/drivers/dronecan driver. Produces dronecan_migration/REFERENCE_MAP.md — the architectural map the design phase (P3) engineers from.',
  phases: [
    { title: 'Read', detail: '8 parallel readers over AP_DroneCAN / cyphal / uavcan subsystems' },
    { title: 'Synthesize', detail: 'merge structured slices into dronecan_migration/REFERENCE_MAP.md' },
  ],
}

const PX4 = '/home/jake/code/jake/PX4-Autopilot'
const AP = '/home/jake/code/jake/ardupilot'

// Each reader returns this structured map of one slice of one reference codebase.
const SLICE_SCHEMA = {
  type: 'object',
  properties: {
    slice: { type: 'string' },
    summary: { type: 'string', description: '3-5 sentence architectural summary of this slice' },
    keyFiles: {
      type: 'array',
      items: {
        type: 'object',
        properties: { path: { type: 'string' }, role: { type: 'string' } },
        required: ['path', 'role'],
      },
    },
    keySymbols: {
      type: 'array',
      description: 'classes / functions / structs that matter, each with a path:line location',
      items: {
        type: 'object',
        properties: { name: { type: 'string' }, location: { type: 'string' }, role: { type: 'string' } },
        required: ['name', 'location', 'role'],
      },
    },
    apiOrDataFlow: {
      type: 'array',
      description: 'ordered steps of the key call-surface or end-to-end data flow; each step cites a path:line',
      items: { type: 'string' },
    },
    reuseDisposition: {
      type: 'array',
      description: 'for each notable element, how the new dronecan driver should treat it',
      items: {
        type: 'object',
        properties: {
          item: { type: 'string' },
          disposition: { type: 'string', enum: ['reuse-as-is', 'rewrite-interior', 'port-mapping', 'do-not-copy', 'reference-only'] },
          note: { type: 'string' },
        },
        required: ['item', 'disposition'],
      },
    },
    gotchas: { type: 'array', items: { type: 'string' }, description: 'traps, footguns, version-specific behaviors' },
    openQuestionsTouched: { type: 'array', items: { type: 'string' }, description: 'which §8 open questions this informs and how' },
  },
  required: ['slice', 'summary', 'keyFiles', 'keySymbols', 'apiOrDataFlow', 'reuseDisposition', 'gotchas'],
}

const PREAMBLE = `You are one reader in a clean-room reference-mapping of the DroneCAN stack, preparing PX4 to replace its libuavcan DroneCAN-v0 driver with a libcanard-based src/drivers/dronecan/ driver. PX4 root = ${PX4}. ArduPilot root = ${AP}. Use Read/Grep/Bash to read the exact files below and CONFIRM every line number you cite (grep for the symbol — paths in this prompt may have drifted). Be precise and terse; your output is reference material an engineer holds open while writing code. Do NOT copy ArduPilot plumbing — map BEHAVIOR. Fill the schema for your slice only.`

const READERS = [
  {
    label: 'AP-1 legacy-libcanard-core',
    prompt: `${PREAMBLE}

SLICE "AP-1: legacy libcanard C API + ArduPilot's CanardInterface wrapper".
Read: ${AP}/libraries/AP_DroneCAN/AP_Canard_iface.h and AP_Canard_iface.cpp; the vendored legacy libcanard ${AP}/modules/DroneCAN/libcanard/canard.h (+ canard.c if present).
Extract precisely (path:line):
- canardInit(&ins, pool, size, onTransferReception, shouldAcceptTransfer, user_ref): the two callbacks registered at init and how the 'this' context is threaded.
- TX: canardBroadcastObj / canardRequestOrRespondObj — args, transfer-id handling, priority, return values.
- RX: canardHandleRxFrame and how it calls back into should_accept / on_reception.
- The single internal memory pool (mem_pool): how it is sized (the POOL param) and that it serves BOTH RX reassembly states AND TX queue items.
- process()/processTx/processRx pumping; any immediate-flush low-latency TX path (processTx(true)).
This is the EXACT API surface PX4's new DronecanHandle must re-express — PX4 keeps this legacy API but drives it from a ScheduledWorkItem, NOT ArduPilot's thread. In reuseDisposition mark the API calls as the surface to call ('rewrite-interior' of the wrapper) and the thread model as 'do-not-copy'. In gotchas, contrast with cyphal's MODERN api (canardTxPush/canardRxAccept/O1Heap) — fundamentally different. Touches §8 Q5 (memory model).`,
  },
  {
    label: 'AP-2 typed-pubsub-and-dsdlc',
    prompt: `${PREAMBLE}

SLICE "AP-2: typed pub/sub template layer + dronecan_dsdlc codegen".
Read: ${AP}/modules/DroneCAN/libcanard/canard/publisher.h, subscriber.h, handler_list.h (and client.h/service_server.h if present); the generator ${AP}/modules/DroneCAN/dronecan_dsdlc/ (the EmPy templates + driver script); a sample generated header/source if you can find one under build/.
Extract precisely (path:line):
- Canard::Publisher<T> / Subscriber<T> / Client / Server: how they register/link (HandlerList::head[CANARD_NUM_HANDLERS][8] indexed by [driver_index][msgid % 8]), and how broadcast()/handle_message() wrap the C codecs.
- The generated cxx_iface trait (static constexpr ID/SIGNATURE/MAX_SIZE + encode/decode fn-ptrs) and how the templates consume it to stay thin.
- dronecan_dsdlc output contract per message: a plain struct, <type>_encode(msg, buf[, tao]), <type>_decode(const CanardRxTransfer*, msg), #defines for _ID/_SIGNATURE/_MAX_SIZE, and (under DRONECAN_CXX_WRAPPERS) the cxx_iface trait.
GOTCHA to call out loudly: <type>_decode returns TRUE on FAILURE. This is the §8 Q1 "typed codec layer" decision (highest-leverage) — capture exactly how thin the template veneer is and what it costs, so the design can choose AP's templates vs a lighter PX4 shim. reuseDisposition: classify the templates + the cxx_iface trait + the generator. Touches §8 Q1.`,
  },
  {
    label: 'AP-3 dna-and-dataflow',
    prompt: `${PREAMBLE}

SLICE "AP-3: DNA-v0 server + end-to-end data-flow traces".
Read: ${AP}/libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.h/.cpp; AP_DroneCAN.h/.cpp (focus the message handlers); AP_GPS_DroneCAN.cpp (the Fix2 trampoline).
Extract precisely (path:line):
- DNA server: the two-step UID allocation (handle_allocation), node monitoring via periodic GetNodeInfo, the node-ID↔unique-ID DB persistence (StorageManager / Bitmask<128>). Summarize the behavior PX4 must reproduce.
- Trace 1 (incoming): gnss.Fix2 → libcanard reassembly → Subscriber::handle_message decode → AP_GPS_DroneCAN::handle_fix2_msg_trampoline → backend resolved by source node-ID → GPS_State fill. Give the ordered steps with file:line.
- Trace 2 (outgoing, low-latency): SRV_send_esc() packs esc.RawCommand and immediately flushes TX (processTx(true)) for low latency. Give the ordered steps with file:line.
- Trace 3 (incoming, handled in-node): esc.Status handled inside AP_DroneCAN::handle_ESC_status.
These traces are the behavioral template for the PX4 decode→uORB-publish / uORB-subscribe→encode bridges. reuseDisposition: DNA behavior = 'port-mapping' (reproduce behavior, not code); traces = 'reference-only'. Touches §8 Q3 (DNA-server source).`,
  },
  {
    label: 'CY-1 canardhandle-and-media',
    prompt: `${PREAMBLE}

SLICE "CY-1: cyphal's CanardHandle wrapper + the reusable CAN media layer".
Read: ${PX4}/src/drivers/cyphal/CanardHandle.hpp/.cpp; CanardInterface.hpp and the concrete CanardSocketCAN / CanardNuttXCDev backends.
Extract precisely (path:line):
- CanardHandle: what it owns (the libcanard instance + TX queue + O1Heap allocator), and the MODERN API it drives — canardInit(&alloc,&free), canardTxPush/Peek/Pop, canardRxAccept (poll/pull), canardRxSubscribe. Note heap-alloc of RX payloads / TX frames from the 8 KB O1Heap arena and that the app must free them.
- CanardInterface + CanardSocketCAN/CanardNuttXCDev: the media abstraction (raw CAN frames), selected by CONFIG_NET_CAN vs CONFIG_CAN.
KEY DESIGN OUTPUT: classify in reuseDisposition — the media layer (CanardInterface/SocketCAN/NuttXCDev) is **reuse-as-is** (protocol-version-agnostic raw frames); the CanardHandle SHELL is reused but its INTERIOR is **rewrite-interior** to the legacy v0 API (AP-1). Make the as-is vs rewrite boundary explicit and precise. Touches §8 Q5 (memory: O1Heap vs legacy internal pool).`,
  },
  {
    label: 'CY-2 uorb-bridge-and-scheduling',
    prompt: `${PREAMBLE}

SLICE "CY-2: cyphal's uORB bridge pattern + work-queue scheduling + codegen/startup wiring".
Read: ${PX4}/src/drivers/cyphal/Cyphal.hpp/.cpp (the ScheduledWorkItem Run loop); Subscribers/udral/Battery.hpp + Subscribers/uORB/uorb_subscriber.hpp; Publishers/uORB/uorb_publisher.hpp + Publishers/udral/Gnss.hpp; CMakeLists.txt (the nnvg/Nunavut codegen step); Kconfig; and ROMFS .../rcS around the UAVCAN/Cyphal mutual-exclusion lines.
Extract precisely (path:line):
- Subscriber pattern: canardRxAccept → user_reference cast to UavcanBaseSubscriber* → callback() runs <type>_deserialize_ then uORB::PublicationMulti<T>::publish(). THE reusable crown jewel — this decode→orb-publish shape is version-independent and the v0 bridges mirror it exactly.
- Publisher pattern: uORB::Subscription::update() → <type>_serialize_ → CanardHandle::TxPush(...).
- Scheduling: ScheduledWorkItem on wq:uavcan, ScheduleOnInterval(3 ms) complemented by CAN/uORB events; Run() locks _node_mutex, lazy init, on parameter_update reconfigures pub/sub, sends periodic node msgs, then transmit(); receive(); transmit(); to drain TX / process RX / flush.
- Codegen: the CMake-configure-time nnvg step (the v0 driver will swap in dronecan_dsdlc analogously).
- Startup: CONFIG_DRIVERS_CYPHAL gating + the rcS mutual exclusion.
reuseDisposition: bridge pattern + scheduling + WorkItem structure = **reuse-as-is** (mirror the shape); Nunavut codegen wiring = **port-mapping** (swap for dronecan_dsdlc). Touches §8 Q1 (codec layer) and the §6.2/§6.4 reuse decisions.`,
  },
  {
    label: 'UA-1 uavcannode-lifecycle',
    prompt: `${PREAMBLE}

SLICE "UA-1: current libuavcan UavcanNode lifecycle / scheduling / CLI / memory / params" (this is the FEATURE-PARITY reference — the replacement must reproduce this). The exhaustive per-sensor field maps are extracted by a SEPARATE workflow (P2) — do NOT enumerate field mappings here; map the node-level skeleton.
Read: ${PX4}/src/drivers/uavcan/uavcan_main.hpp/.cpp; uavcan_params.yaml; module.yaml; Kconfig; the rcS boot line.
Extract precisely (path:line):
- Lifecycle: UavcanNode : px4::ScheduledWorkItem, ModuleParams — singleton, wq:uavcan, ScheduleOnInterval(3 ms) PLUS the CAN-RX-IRQ ScheduleNow() wakeup (uavcan_main.cpp ~485-497).
- CLI surface: uavcan {start|status|stop|shrink|update|param {set|get|list|save} <node>|reset <node>}.
- Memory: HeapBasedPoolAllocator, block size 48, soft 250 / hard 500 blocks; what the 'shrink' command does.
- Param set + semantics: UAVCAN_ENABLE (0/1/2/3 tiers), UAVCAN_NODE_ID, UAVCAN_BITRATE, UAVCAN_ESC_IFACE, UAVCAN_SUB_*, UAVCAN_PUB_*, UAVCAN_EC_*/UAVCAN_SV_*; the rcS keying (if param greater UAVCAN_ENABLE 0; then uavcan start).
reuseDisposition: lifecycle/scheduling/CLI = 'port-mapping' (reproduce on the new driver); param names = 'port-mapping' (rename UAVCAN_*→DC_*, decision 4); memory model = 'reference-only' (legacy libcanard uses its own internal pool — Q5). Touches §8 Q5, Q6.`,
  },
  {
    label: 'UA-2 uavcan-services',
    prompt: `${PREAMBLE}

SLICE "UA-2: current uavcan services & node management" (feature-parity reference; decision 5 keeps ALL of this).
Read: ${PX4}/src/drivers/uavcan/uavcan_servers.hpp/.cpp; node_info.hpp/.cpp; and the MAVLink remote-param bridge state machine in uavcan_main.cpp (~771-970).
Extract precisely (path:line):
- UavcanServers (gated UAVCAN_ENABLE>1): the centralized DNA server + BasicFileServer + FirmwareUpdateTrigger + FW DB under /fs/microsd/ufw. Capture exactly what "SD-card firmware serving" does — decision 5 makes this mandatory.
- NodeInfoRetriever; NodeStatusMonitor → dronecan_node_status; NodeInfoPublisher → device_information; can_interface_status; GlobalTimeSync master/slave; log forwarding (debug.LogMessage → px4_log).
- The MAVLink remote-param bridge: the GetSet / ExecuteOpcode / RestartNode state machine driven from Run(), and the uORB topics it bridges (uavcan_parameter_request/value, vehicle_command/_ack).
reuseDisposition: all = 'port-mapping' (reproduce behavior on libcanard). Flag which pieces are large/v0-specific and belong in P6 (MAVLink param bridge) vs the P4 core slice (DNA + SD-card FW server). Touches §8 Q3 (DNA source), Q4 (MAVLink bridge sequencing).`,
  },
  {
    label: 'UA-3 actuators-and-aux',
    prompt: `${PREAMBLE}

SLICE "UA-3: current uavcan actuator outputs + aux controllers" (feature-parity reference).
Read: ${PX4}/src/drivers/uavcan/actuators/{esc,servo,hardpoint}.cpp/.hpp; and the aux controllers arming_status, beep, rgbled, safety_state, remoteid, logmessage (.cpp/.hpp) under src/drivers/uavcan/.
Extract precisely (path:line):
- ESC: UavcanEscController — out esc.RawCommand @≤400 Hz; in esc.Status/StatusExtended → esc_status + failure parsing. Note it runs as an independent MixingOutput/OutputModuleInterface WorkItem with UAVCAN_EC_* scaling. (ESC is in the P4 core slice.)
- Servo: UavcanServoController — actuator.ArrayCommand @50 Hz, UAVCAN_SV_* scaling. Hardpoint: UavcanHardpointController — hardpoint.Command.
- Aux controllers (each CONFIG_UAVCAN_*-gated): ArmingStatus; Beep (tune_control→BeepCommand); RGB LED (LightsCommand); SafetyState; RemoteID (bidirectional Open Drone ID — large, v0-specific, P6); GNSS RTCM/MovingBaseline injection + PPK gps_dump.
reuseDisposition: ESC/servo/hardpoint MixingOutput path = 'reuse-as-is' (the §6.5 decision keeps the current MixingOutput path); aux controllers = 'port-mapping'. Flag RemoteID as P6. Touches §8 Q4, Q7 (ESC latency).`,
  },
]

phase('Read')
log(`P1 reference map: fanning out ${READERS.length} readers over AP_DroneCAN / cyphal / uavcan`)
// Barrier is correct here: the synthesis step needs ALL slices at once to build one coherent map.
const slices = (await parallel(READERS.map(r => () =>
  agent(r.prompt, { label: r.label, phase: 'Read', schema: SLICE_SCHEMA })
))).filter(Boolean)
log(`Collected ${slices.length}/${READERS.length} slices`)

phase('Synthesize')
const outPath = `${PX4}/dronecan_migration/REFERENCE_MAP.md`
const synthPrompt = `You are the synthesis step of the DroneCAN libcanard migration reference-mapping. Below is structured JSON from ${slices.length} reader agents. Sources: ArduPilot AP_DroneCAN = behavioral / legacy-libcanard reference; PX4 cyphal = PX4-idiom / libcanard-in-PX4 reference; PX4 uavcan = feature-parity reference.

Write ONE coherent markdown document to ${outPath} using the Write tool (overwrite if present). Structure:

1. **Purpose & how to use** — 1 paragraph: this is the engineering input for designing src/drivers/dronecan; the exhaustive per-bridge field maps live in the separate ACCEPTANCE_SPEC (P2).
2. **The legacy libcanard API surface we must re-express** (AP-1, AP-2) — the exact C calls DronecanHandle wraps, with path:line; an explicit contrast table vs cyphal's modern API.
3. **PX4 shapes we reuse verbatim** (CY-1, CY-2) — media layer, the decode→uORB-publish / uORB-subscribe→encode bridge pattern, ScheduledWorkItem scheduling, codegen wiring; cite cyphal path:line.
4. **PX4 shapes we rewrite the interior of** — the CanardHandle shell (cyphal) with legacy guts (AP-1); make the as-is vs rewrite boundary explicit.
5. **Feature inventory to preserve** (UA-1, UA-2, UA-3) — pointer-level only; link forward to ACCEPTANCE_SPEC for field maps. Tag each item P4-slice / P5 / P6.
6. **Consolidated reuse/rewrite/port disposition table** — merge every reuseDisposition across all slices into one table: element | source | disposition | note.
7. **Gotchas** — merged + deduped. Call out loudly: dronecan_dsdlc <type>_decode returns TRUE on FAILURE; legacy single internal pool vs O1Heap; the canard* symbol collision (resolved by decision 2).
8. **Bearing on the §8 open questions** — one short subsection per open question that any slice touched, with the concrete recommendation the evidence supports (esp. Q1 typed-codec layer).

Preserve every path:line citation verbatim. Terse, dense, engineer-facing. After writing, return a 3-line summary: the path, the section count, and the 3 most consequential findings for the design.

STRUCTURED READER OUTPUT:
${JSON.stringify(slices, null, 2)}`

const result = await agent(synthPrompt, { label: 'synthesize → REFERENCE_MAP.md', phase: 'Synthesize' })
log('P1 complete → dronecan_migration/REFERENCE_MAP.md')
return result
