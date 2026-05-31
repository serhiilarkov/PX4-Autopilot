export const meta = {
  name: 'dronecan-port-bridge',
  description: 'P5/Migrate: port a list of sensor bridges from the current libuavcan driver to the new libcanard src/drivers/dronecan driver, one worktree per bridge, each verified field-for-field against ACCEPTANCE_SPEC.md',
  whenToUse: 'Run AFTER P4 lands the vertical slice (so an example dronecan bridge exists) AND P2 produced dronecan_migration/ACCEPTANCE_SPEC.md. Pass args {bridges:["Baro","Mag",...], exampleBridge:"GNSS"}. Does NOT build firmware (too heavy in parallel) — run /dronecan-flash-delta or P7 for the integrated build.',
  phases: [
    { title: 'Implement', detail: 'one worktree per bridge: write the dronecan bridge, wire Kconfig/CMake, check_format' },
    { title: 'Verify', detail: 'fresh agent checks each ported bridge field-for-field vs ACCEPTANCE_SPEC' },
  ],
}

const PX4 = '/home/jake/code/jake/PX4-Autopilot'

const IMPL_SCHEMA = {
  type: 'object',
  properties: {
    bridge: { type: 'string' },
    filesWritten: { type: 'array', items: { type: 'string' } },
    kconfigCmakeWired: { type: 'boolean' },
    formatClean: { type: 'boolean' },
    summary: { type: 'string' },
    openItems: { type: 'array', items: { type: 'string' } },
  },
  required: ['bridge', 'filesWritten', 'summary'],
}

const VERDICT_SCHEMA = {
  type: 'object',
  properties: {
    matchesSpec: { type: 'boolean' },
    missingFields: { type: 'array', items: { type: 'string' } },
    incorrectTransforms: { type: 'array', items: { type: 'string' } },
    gateOrRegistrationIssues: { type: 'array', items: { type: 'string' } },
    notes: { type: 'string' },
  },
  required: ['matchesSpec', 'missingFields'],
}

const cfg = args || {}
const bridges = Array.isArray(cfg) ? cfg : (cfg.bridges || [])
const exampleBridge = cfg.exampleBridge || 'GNSS'

if (!bridges.length) {
  log('dronecan-port-bridge: no bridges given. Pass args {bridges:["Baro","Mag",...], exampleBridge:"GNSS"}. Prereqs: P4 vertical slice (an example bridge under src/drivers/dronecan) and dronecan_migration/ACCEPTANCE_SPEC.md must exist.')
  return { ported: [], note: 'no-op: provide args.bridges after P4' }
}

const implementPrompt = (b) =>
  'You are porting ONE sensor bridge from PX4 legacy libuavcan to the new libcanard driver. PX4 root = ' + PX4 + '. You are in an isolated git worktree — edit freely.\n\n' +
  'BRIDGE TO PORT: ' + b + '\n\n' +
  'Read, in order:\n' +
  '1. The current implementation under ' + PX4 + '/src/drivers/uavcan/ (grep for the bridge).\n' +
  '2. Its row in ' + PX4 + '/dronecan_migration/ACCEPTANCE_SPEC.md (the NORMATIVE field map — reproduce it exactly).\n' +
  '3. The example dronecan bridge "' + exampleBridge + '" already implemented under ' + PX4 + '/src/drivers/dronecan/ (copy its shape, registration, and idioms — legacy libcanard codecs + decode->uORB::PublicationMulti::publish).\n' +
  '4. The cyphal subscriber/publisher pattern under ' + PX4 + '/src/drivers/cyphal/ if you need a second reference.\n\n' +
  'Then: write the new bridge files under ' + PX4 + '/src/drivers/dronecan/ mirroring the example bridge; register it; wire the Kconfig CONFIG_DRONECAN_* gate and the CMake entry; and run make check_format on the files you wrote (fix until clean). Do NOT build firmware. Remember dronecan_dsdlc <type>_decode returns TRUE on FAILURE. Fill the schema.'

const verifyPrompt = (b) =>
  'You are adversarially verifying a freshly ported PX4 DroneCAN bridge. PX4 root = ' + PX4 + '.\n\n' +
  'BRIDGE: ' + b + '\n' +
  'Read the new implementation under ' + PX4 + '/src/drivers/dronecan/ and its NORMATIVE row in ' + PX4 + '/dronecan_migration/ACCEPTANCE_SPEC.md. Verify FIELD-BY-FIELD that every uORB field is populated from the correct DroneCAN source with the correct transform/units, that the compile + runtime gates and the redundancy/device-id handling match, and that the subscriber/publisher is actually registered. Assume there IS a bug until you have checked each field. Report matchesSpec only if every spec field is correctly reproduced. Fill the schema.'

phase('Implement')
log('P5: porting ' + bridges.length + ' bridges (example=' + exampleBridge + '), one worktree each')
const results = await pipeline(
  bridges,
  (b) => agent(implementPrompt(b), { label: 'impl:' + b, phase: 'Implement', isolation: 'worktree', schema: IMPL_SCHEMA }),
  (impl, b) => agent(verifyPrompt(b), { label: 'verify:' + b, phase: 'Verify', schema: VERDICT_SCHEMA })
    .then(v => ({ bridge: b, impl, verdict: v }))
)

const done = results.filter(Boolean)
const clean = done.filter(r => r.verdict && r.verdict.matchesSpec)
log('P5 done: ' + clean.length + '/' + done.length + ' bridges match the spec on first pass')
return { ported: done, summary: clean.length + '/' + done.length + ' verified against ACCEPTANCE_SPEC; integrated build is a separate step (/dronecan-flash-delta or P7).' }
