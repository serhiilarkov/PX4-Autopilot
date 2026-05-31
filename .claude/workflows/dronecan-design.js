export const meta = {
  name: 'dronecan-design',
  description: 'P3/Design: draft the new dronecan driver architecture from several independent angles, score each with a judge panel, and synthesize one proposal (DESIGN.md) resolving the open questions — for plan-mode sign-off. No repo changes.',
  whenToUse: 'Run after P1 (REFERENCE_MAP.md) and P2 (ACCEPTANCE_SPEC.md). Produces dronecan_migration/DESIGN.md: the architecture + §8 resolutions + the P4 vertical-slice plan, to review in plan mode before any driver code is written.',
  phases: [
    { title: 'Draft', detail: '4 architects each draft a full architecture from a distinct bias' },
    { title: 'Judge', detail: '3-lens panel scores each draft (flash/Q1, PX4-correctness/risk, maintainability)' },
    { title: 'Synthesize', detail: 'build one proposal from the winner + grafted best ideas -> DESIGN.md' },
  ],
}

const PX4 = '/home/jake/code/jake/PX4-Autopilot'

const DESIGN_SCHEMA = {
  type: 'object',
  properties: {
    angle: { type: 'string' },
    summary: { type: 'string', description: '4-6 sentence overview of this architecture' },
    q1CodecLayer: { type: 'string', description: 'Q1: templates vs non-templated descriptor-driven dispatcher vs hybrid; how thin; why it stays flat (no per-type bloat)' },
    memoryPoolModel: { type: 'string', description: 'Q5: the legacy internal-pool sizing/model; how RX/TX share it; param' },
    dronecanHandle: { type: 'string', description: 'how the legacy libcanard API is wrapped (canardInit + callbacks, broadcast/handleRxFrame); cyphal shell vs rewritten guts' },
    dispatchRegistry: { type: 'string', description: 'how subscribers/handlers register and how RX dispatches to them (per-instance vs static)' },
    moduleStructure: {
      type: 'array', description: 'the src/drivers/dronecan/ file/class layout',
      items: { type: 'object', properties: { path: { type: 'string' }, role: { type: 'string' } }, required: ['path', 'role'] },
    },
    dnaAndFwServer: { type: 'string', description: 'Q3: DNA-v0 allocation + SD-card FW serving approach' },
    schedulingThreading: { type: 'string', description: 'ScheduledWorkItem model, ticks, IRQ wakeup, ESC immediate-flush' },
    codegenWiring: { type: 'string', description: 'how dronecan_dsdlc is wired into CMake (build-time custom_command)' },
    paramMigration: { type: 'string', description: 'Q4/Q6: UAVCAN_*->DC_* migration approach' },
    verticalSlicePlan: { type: 'array', items: { type: 'string' }, description: 'ordered P4 steps to land GNSS+battery+ESC+DNA+SD-FW behind DRIVERS_DRONECAN' },
    openQuestionResolutions: {
      type: 'array', description: 'a position on each §8 open question',
      items: { type: 'object', properties: { question: { type: 'string' }, resolution: { type: 'string' } }, required: ['question', 'resolution'] },
    },
    risks: { type: 'array', items: { type: 'string' } },
  },
  required: ['angle', 'summary', 'q1CodecLayer', 'memoryPoolModel', 'dronecanHandle', 'dispatchRegistry', 'moduleStructure', 'verticalSlicePlan', 'openQuestionResolutions'],
}

const SCORE_SCHEMA = {
  type: 'object',
  properties: {
    lens: { type: 'string' },
    overall: { type: 'number', description: '1-10 overall score for this draft through this lens' },
    strengths: { type: 'array', items: { type: 'string' } },
    weaknesses: { type: 'array', items: { type: 'string' } },
    killers: { type: 'array', items: { type: 'string' }, description: 'fatal flaws (violates a locked decision, misses the flash goal, reproduces a known bug, safety risk)' },
    bestIdeasToGraft: { type: 'array', items: { type: 'string' }, description: 'ideas worth keeping even if this draft loses' },
  },
  required: ['lens', 'overall', 'strengths', 'weaknesses'],
}

const PREAMBLE =
  'You are one of several INDEPENDENT architects proposing a design for PX4 new libcanard-based DroneCAN-v0 driver (src/drivers/dronecan/), replacing the libuavcan src/drivers/uavcan/ to reclaim ~64KB flash. PX4 root = ' + PX4 + '.\n\n' +
  'READ FIRST: dronecan_migration/REFERENCE_MAP.md IN FULL (the architectural map: what to reuse/gut/port, the exact legacy libcanard API surface, the gotchas, draft recommendations for the open questions). Then understand the parity surface your design must support: grep the section headers of dronecan_migration/ACCEPTANCE_SPEC.md and read 2-3 representative bridge sections (GNSS, Battery, ESC) — you do NOT need every field. Skim DRONECAN_LIBCANARD_MIGRATION.md sections 2.1 and 8.\n\n' +
  'LOCKED decisions you MUST respect (do not re-litigate): (1) vendor legacy dronecan/libcanard + dronecan_dsdlc as submodules; (2) dronecan is mutually exclusive with cyphal via Kconfig; (3) new module dronecan, mutually exclusive with old uavcan, board migration = one Kconfig flip; (4) params UAVCAN_*->DC_* with an idempotent startup migration; (5) preserve DNA + SD-card FW serving.\n\n' +
  'Produce a COMPLETE, COHERENT architecture. Take a concrete position on EVERY schema field — especially Q1 (the typed codec layer) and Q5 (memory/pool). Your design must support ALL 18 bridges/actuators in the acceptance spec AND the full service set (DNA + SD-card FW server, node monitor, time sync, log forward, MAVLink param bridge). Be specific: name classes, files, and the exact legacy libcanard calls. Cite REFERENCE_MAP path:line where it grounds a choice.\n\n' +
  'YOUR ANGLE (optimize hard for this bias, even at the expense of others — diversity across architects is the point): '

const ANGLES = [
  { key: 'flash-minimal', bias: 'FLASH-MINIMAL. Optimize hardest for the ~64KB reclaim. Push the codec/dispatch veneer as thin as physically possible (minimum per-type machine code); accept more manual/explicit wiring if it saves flash. Justify every byte-costly abstraction and estimate where the bytes go.' },
  { key: 'cyphal-symmetric', bias: 'CYPHAL-SYMMETRIC. Maximize structural reuse from src/drivers/cyphal (lowest integration risk, closest PX4 idiom). Mirror its CanardHandle / BaseSubscriber / Publication-Subscription-Manager / ScheduledWorkItem shapes as literally as the legacy v0 API allows; deviate ONLY where the v0 callback/pool model forces it, and say exactly where.' },
  { key: 'ardupilot-faithful', bias: 'ARDUPILOT-FAITHFUL. Minimize protocol/behavioral risk by staying closest to AP_DroneCAN proven legacy-libcanard usage (the DNA FSM, the cxx_iface trait seam, the immediate-flush ESC path, the should_accept/on_reception contract), re-expressed in PX4 idioms. Prefer proven behavior over novel structure.' },
  { key: 'maintainability-first', bias: 'MAINTAINABILITY-FIRST. Optimize for the next maintainer: clearest layering, strongest testability (SITL + unit), least template/macro magic, explicit error handling (especially the decode-returns-TRUE-on-failure trap and pool exhaustion). Willing to spend a little flash for clarity, but stay within the ~64KB goal and say what it costs.' },
]

const JUDGES = [
  { key: 'flash-q1', prompt: 'Judge this dronecan architecture on FLASH & Q1. Will it actually reclaim ~64KB and keep the per-type codec layer FLAT (no libuavcan-style per-type explosion)? Flag every abstraction that reintroduces per-type bloat: template instantiation per message type, vtable-per-type, function-pointer trait MEMBERS that block inlining, a MAX_SIZE stack/heap buffer per type, malloc-per-subscription. A design that keeps AP Publisher/Subscriber<T> templates wholesale should score low here. Score 1-10.' },
  { key: 'px4-correctness-risk', prompt: 'Judge this dronecan architecture on PX4-CORRECTNESS & RISK. Does it correctly reuse the cyphal media / uORB PublicationMulti / WorkQueue idioms, respect ALL FIVE locked decisions, and explicitly handle the REFERENCE_MAP gotchas (decode returns TRUE on failure; ONE pool shared by RX reassembly + TX; should_accept MUST set out_data_type_signature; anonymous-node TX size limits; the canard* symbol collision)? Any protocol-correctness or flight-safety risk? A design that ignores the shared-pool starvation or the decode-negation is a killer. Score 1-10.' },
  { key: 'maintainability-feasibility', prompt: 'Judge this dronecan architecture on MAINTAINABILITY & FEASIBILITY. Clarity of layering, testability (can the codecs/dispatch be unit-tested off-target? SITL?), whether the P4 vertical slice (GNSS+battery+ESC+DNA+SD-FW) is actually buildable as described, and whether the design demonstrably supports ALL 18 bridges + the full service set (esp. the hard ones: Battery 4-mode, MAVLink param bridge, SD-card FW server). Score 1-10.' },
]

phase('Draft')
log('P3 design: ' + ANGLES.length + ' architects drafting, then a ' + JUDGES.length + '-lens judge panel each')
const results = await pipeline(
  ANGLES,
  (a) => agent(PREAMBLE + a.bias, { label: 'draft:' + a.key, phase: 'Draft', schema: DESIGN_SCHEMA }),
  (draft, a) => parallel(JUDGES.map(j => () =>
    agent(
      'You are judging ONE candidate architecture for PX4 new dronecan driver. PX4 root = ' + PX4 + '. Read dronecan_migration/REFERENCE_MAP.md for the gotchas/constraints if you need to verify a claim. ' + j.prompt +
      '\n\nBe specific and cite the part of the draft you are judging. List killers (fatal flaws) separately from weaknesses, and bestIdeasToGraft (worth keeping even if this draft loses).\n\nCANDIDATE ARCHITECTURE (angle: ' + a.key + '):\n' + JSON.stringify(draft, null, 2),
      { label: 'judge:' + a.key + '/' + j.key, phase: 'Judge', schema: SCORE_SCHEMA }
    ).then(score => ({ ...score, judge: j.key }))
  )).then(scores => ({ angle: a.key, draft, scores, aggregate: scores.reduce((s, x) => s + (x.overall || 0), 0) }))
)

const judged = results.filter(Boolean)
judged.forEach(r => log('  ' + r.angle + ': aggregate ' + r.aggregate + '/' + (JUDGES.length * 10) +
  (r.scores.some(s => (s.killers || []).length) ? ' (has killer flags)' : '')))

phase('Synthesize')
const outPath = PX4 + '/dronecan_migration/DESIGN.md'
const synthPrompt =
  'You are the synthesis step of the dronecan design phase (P3). Below are ' + judged.length + ' candidate architectures, each with its judge-panel scores (3 lenses: flash/Q1, PX4-correctness/risk, maintainability/feasibility), an aggregate, and killer flags. PX4 root = ' + PX4 + '.\n\n' +
  'Build ONE proposal: take the highest-aggregate draft WITHOUT an unresolved killer as the backbone, then GRAFT the best ideas from the others (use each draft bestIdeasToGraft and strengths). Any killer flagged by the PX4-correctness/risk lens (locked-decision violation, ignored shared-pool starvation, missing decode-negation, safety risk) MUST be resolved in the final design, not carried.\n\n' +
  'Write dronecan_migration/DESIGN.md (Write tool, overwrite). Structure: (1) Chosen architecture + why (backbone angle + which grafts, with the aggregate scores table); (2) Module/file structure of src/drivers/dronecan/; (3) DronecanHandle design — legacy API wrapping + the pool model (Q5); (4) Codec & dispatch design (Q1) — concrete, with the flatness argument; (5) DNA + SD-card FW server (Q3); (6) Scheduling/threading + ESC immediate-flush (Q7); (7) Param migration (Q4/Q6); (8) The P4 vertical-slice implementation plan (ordered, buildable steps); (9) What the Q1 measurement SPIKE must confirm before the design is locked; (10) Decision log: each §8 question -> the locked resolution + which draft/judge it came from; (11) Top open risks for plan-mode review. Preserve REFERENCE_MAP path:line citations. Terse, engineer-facing, decisive.\n\n' +
  'After writing, return: the backbone angle, the aggregate-score table, the Q1/Q3/Q5 resolutions in one line each, and the top 3 risks to raise at the plan-mode gate.\n\n' +
  'CANDIDATES + SCORES:\n' + JSON.stringify(judged, null, 2)

const result = await agent(synthPrompt, { label: 'synthesize -> DESIGN.md', phase: 'Synthesize' })
log('P3 design draft complete -> dronecan_migration/DESIGN.md (review in plan mode)')
return result
