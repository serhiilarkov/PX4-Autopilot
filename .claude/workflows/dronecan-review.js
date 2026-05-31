export const meta = {
  name: 'dronecan-review',
  description: 'P7/Verify: multi-dimension adversarial review of the new src/drivers/dronecan driver against the acceptance spec, plus a flash-size gate confirming the ~64 KB reclaim. Each finding is independently verified before it is reported.',
  whenToUse: 'Run after the dronecan driver (or a meaningful slice) is implemented. Reviews spec-parity, decode error handling, memory/pool, scheduling, param migration, and DNA/FW-server parity; confirms the flash delta on a target board. Pass args {board:"ark_fmu-v6x_default"}.',
  phases: [
    { title: 'Review', detail: 'one reviewer per dimension -> candidate findings' },
    { title: 'Verify', detail: 'adversarially verify each finding (refute by default)' },
    { title: 'Flash gate', detail: 'build + size measurement vs the libuavcan baseline' },
    { title: 'Report', detail: 'synthesize confirmed findings -> dronecan_migration/REVIEW.md' },
  ],
}

const PX4 = '/home/jake/code/jake/PX4-Autopilot'
const board = (args && args.board) || 'ark_fmu-v6x_default'

const FINDINGS_SCHEMA = {
  type: 'object',
  properties: {
    dimension: { type: 'string' },
    findings: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          title: { type: 'string' },
          location: { type: 'string', description: 'path:line' },
          severity: { type: 'string', enum: ['blocker', 'major', 'minor', 'nit'] },
          detail: { type: 'string' },
        },
        required: ['title', 'location', 'severity', 'detail'],
      },
    },
  },
  required: ['dimension', 'findings'],
}

const VERDICT_SCHEMA = {
  type: 'object',
  properties: {
    isReal: { type: 'boolean' },
    reason: { type: 'string' },
  },
  required: ['isReal', 'reason'],
}

const DIMENSIONS = [
  { key: 'spec-parity', focus: 'Compare every implemented dronecan bridge / actuator against its NORMATIVE row in dronecan_migration/ACCEPTANCE_SPEC.md. Report any missing or incorrect field mapping, wrong transform/units, missing topic instance, or missing compile/runtime gate.' },
  { key: 'decode-error-handling', focus: 'dronecan_dsdlc <type>_decode returns TRUE on FAILURE. Audit EVERY decode call site in src/drivers/dronecan: report any site that treats true as success, ignores the return, or uses a partially-decoded struct.' },
  { key: 'memory-pool', focus: 'Audit the legacy libcanard internal pool sizing vs the old HeapBasedPoolAllocator soft/hard block budget. Report under/over-sizing, missing free of RX payload / TX frame items, and pool-exhaustion handling. Cross-check the DC_* pool/memory param against the old behavior.' },
  { key: 'scheduling', focus: 'Audit the ScheduledWorkItem Run() loop: _node_mutex discipline, the transmit/receive/transmit drain order, the CAN-RX-IRQ ScheduleNow() wakeup, and the ESC immediate-flush low-latency path. Report races, missed wakeups, or latency regressions vs the current uavcan driver.' },
  { key: 'param-migration', focus: 'Audit the UAVCAN_* -> DC_* startup migration function: idempotency (runs once, no-op thereafter), completeness of the name map, the 16-char name limit, and that semantics are preserved. Report any param that is dropped, mis-mapped, or re-migrated on every boot.' },
  { key: 'dna-fw-server', focus: 'Audit DNA-v0 allocation and the SD-card firmware/file serving (BasicFileServer + FirmwareUpdateTrigger + FW DB under /fs/microsd/ufw) for behavioral parity with the old UavcanServers (decision 5 keeps both). Report any allocation or FW-serving gap.' },
]

const reviewPrompt = (d) =>
  'You are reviewing the new PX4 libcanard DroneCAN driver. PX4 root = ' + PX4 + '. Use Read/Grep/Bash; cite path:line for every finding.\n\n' +
  'REVIEW DIMENSION: ' + d.key + '\n' + d.focus + '\n\n' +
  'Read the relevant code under ' + PX4 + '/src/drivers/dronecan/ (and ' + PX4 + '/dronecan_migration/ACCEPTANCE_SPEC.md / the current ' + PX4 + '/src/drivers/uavcan/ for parity reference). Report only real, specific, located findings — not style. Fill the schema (empty findings array is a valid, good result).'

const verifyFindingPrompt = (dim, f) =>
  'Adversarially verify ONE code-review finding about the PX4 dronecan driver. PX4 root = ' + PX4 + '. Default to isReal=false unless you can confirm it at the cited location by reading the code.\n\n' +
  'DIMENSION: ' + dim + '\n' +
  'TITLE: ' + f.title + '\n' +
  'LOCATION: ' + f.location + '\n' +
  'CLAIM: ' + f.detail + '\n\n' +
  'Read the code at that location and decide: is this a genuine defect (isReal=true) or a false positive / already-handled / misread (isReal=false)? Give a one-sentence reason citing what you saw.'

phase('Review')
log('P7 review of src/drivers/dronecan across ' + DIMENSIONS.length + ' dimensions; flash board=' + board)
const reviewed = await pipeline(
  DIMENSIONS,
  (d) => agent(reviewPrompt(d), { label: 'review:' + d.key, phase: 'Review', schema: FINDINGS_SCHEMA }),
  (rev) => parallel((rev.findings || []).map(f => () =>
    agent(verifyFindingPrompt(rev.dimension, f), { label: 'verify:' + String(f.location || f.title).slice(0, 28), phase: 'Verify', schema: VERDICT_SCHEMA })
      .then(v => ({ dimension: rev.dimension, title: f.title, location: f.location, severity: f.severity, detail: f.detail, verdict: v }))
  ))
)
const confirmed = reviewed.flat().filter(Boolean).filter(f => f.verdict && f.verdict.isReal)
log('Confirmed ' + confirmed.length + ' findings after adversarial verification')

phase('Flash gate')
const flashPrompt =
  'Measure the DroneCAN flash footprint for board ' + board + ' and confirm the libcanard migration is reclaiming flash. PX4 root = ' + PX4 + '.\n' +
  'Follow the methodology in DRONECAN_LIBCANARD_MIGRATION.md section 9.1: build the board, then run arm-none-eabi-nm --print-size --radix=d --demangle on the ELF, keep text/rodata symbols, dedup by address, and bucket by demangled name. Report the total dronecan/canard subsystem .text+.rodata, and (if a libuavcan baseline build is available or referenced) the delta vs the ~124 KB libuavcan subsystem. State whether the ~64 KB reclaim target is on track. Return a concise numeric summary (do not write files).'
const flash = await agent(flashPrompt, { label: 'flash-gate:' + board, phase: 'Flash gate' })

phase('Report')
const reportPrompt =
  'Write the DroneCAN driver review report to ' + PX4 + '/dronecan_migration/REVIEW.md using the Write tool (overwrite). Inputs below: confirmed findings (already adversarially verified) and the flash measurement.\n' +
  'Structure: (1) verdict line (ship / fix-then-ship / blocked) + flash status; (2) blockers; (3) majors; (4) minors/nits; (5) the flash measurement table. Each finding: title, path:line, one-line fix. Be terse. After writing, return the verdict line and the counts by severity.\n\n' +
  'CONFIRMED FINDINGS:\n' + JSON.stringify(confirmed, null, 2) + '\n\nFLASH MEASUREMENT:\n' + flash
const report = await agent(reportPrompt, { label: 'review report -> REVIEW.md', phase: 'Report' })
log('P7 complete -> dronecan_migration/REVIEW.md')
return report
