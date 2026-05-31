export const meta = {
  name: 'dronecan-acceptance-spec',
  description: 'P2/Acceptance: extract the exact DroneCAN-message-to-uORB field mappings (plus compile/runtime gates, redundancy, device-id encoding) from the current libuavcan UavcanNode bridges and actuators, producing ACCEPTANCE_SPEC.md — the test oracle for porting',
  whenToUse: 'Run after P1. Produces dronecan_migration/ACCEPTANCE_SPEC.md: one precise field-map table per sensor bridge + actuator, used to verify every ported dronecan bridge in P5/P7.',
  phases: [
    { title: 'Extract', detail: 'one reader per bridge / actuator -> structured field map' },
    { title: 'Synthesize', detail: 'merge into dronecan_migration/ACCEPTANCE_SPEC.md' },
  ],
}

const PX4 = '/home/jake/code/jake/PX4-Autopilot'

const FIELD_MAP_SCHEMA = {
  type: 'object',
  properties: {
    bridge: { type: 'string' },
    sourceFiles: { type: 'array', items: { type: 'string' }, description: 'path:line of the implementing file(s) under src/drivers/uavcan' },
    dronecanMessages: { type: 'array', items: { type: 'string' } },
    uorbTopic: { type: 'string' },
    direction: { type: 'string', enum: ['incoming', 'outgoing', 'bidirectional'] },
    fieldMappings: {
      type: 'array',
      description: 'every field-level mapping between the DroneCAN message and the uORB topic',
      items: {
        type: 'object',
        properties: {
          uorbField: { type: 'string' },
          source: { type: 'string', description: 'DroneCAN message field or expression it derives from' },
          transform: { type: 'string', description: 'unit conversion / scaling / remap, or "direct"' },
        },
        required: ['uorbField', 'source'],
      },
    },
    compileGate: { type: 'string', description: 'CONFIG_UAVCAN_* compile gate' },
    runtimeGate: { type: 'string', description: 'UAVCAN_SUB_* / UAVCAN_PUB_* runtime param gate' },
    redundancy: { type: 'string', description: 'multi-instance / per-source-node redundancy handling' },
    deviceIdEncoding: { type: 'string', description: 'how the uORB device_id is built' },
    notes: { type: 'array', items: { type: 'string' } },
  },
  required: ['bridge', 'sourceFiles', 'dronecanMessages', 'uorbTopic', 'direction', 'fieldMappings'],
}

const BRIDGES = [
  { name: 'GNSS', msg: 'gnss.Fix / gnss.Fix2 / gnss.Auxiliary (+ardupilot.gnss.RelPosHeading)', topic: 'sensor_gps', dir: 'incoming', loc: 'sensors/' },
  { name: 'GNSS relative', msg: 'ardupilot.gnss.RelPosHeading', topic: 'sensor_gnss_relative', dir: 'incoming', loc: 'sensors/' },
  { name: 'Baro', msg: 'air_data.StaticPressure / StaticTemperature / RawAirData', topic: 'sensor_baro', dir: 'incoming', loc: 'sensors/' },
  { name: 'Mag', msg: 'ahrs.MagneticFieldStrength / MagneticFieldStrength2', topic: 'sensor_mag', dir: 'incoming', loc: 'sensors/' },
  { name: 'Accel', msg: 'ahrs.RawIMU', topic: 'sensor_accel', dir: 'incoming', loc: 'sensors/' },
  { name: 'Gyro', msg: 'ahrs.RawIMU', topic: 'sensor_gyro', dir: 'incoming', loc: 'sensors/' },
  { name: 'Airspeed', msg: 'air_data.IndicatedAirspeed / TrueAirspeed / StaticTemperature', topic: 'airspeed', dir: 'incoming', loc: 'sensors/' },
  { name: 'Differential pressure', msg: 'air_data.RawAirData', topic: 'differential_pressure', dir: 'incoming', loc: 'sensors/' },
  { name: 'Battery', msg: 'power.BatteryInfo (+BatteryInfoAux, cuav CBAT)', topic: 'battery_status / battery_info', dir: 'incoming', loc: 'sensors/' },
  { name: 'Optical flow', msg: 'com.hex.equipment.flow.Measurement', topic: 'sensor_optical_flow', dir: 'incoming', loc: 'sensors/' },
  { name: 'Rangefinder', msg: 'range_sensor.Measurement', topic: 'distance_sensor', dir: 'incoming', loc: 'sensors/' },
  { name: 'Hygrometer', msg: 'dronecan.sensors.hygrometer.Hygrometer', topic: 'sensor_hygrometer', dir: 'incoming', loc: 'sensors/' },
  { name: 'Fuel tank', msg: 'ice.FuelTankStatus', topic: 'fuel_tank_status', dir: 'incoming', loc: 'sensors/' },
  { name: 'ICE status', msg: 'ice.reciprocating.Status', topic: 'internal_combustion_engine_status', dir: 'incoming', loc: 'sensors/' },
  { name: 'Safety button', msg: 'ardupilot.indication.Button', topic: 'button_event', dir: 'incoming', loc: 'sensors/' },
  { name: 'ESC', msg: 'out esc.RawCommand; in esc.Status / esc.StatusExtended', topic: 'esc_status (+ actuator_motors/actuator_outputs in)', dir: 'bidirectional', loc: 'actuators/' },
  { name: 'Servo', msg: 'actuator.ArrayCommand', topic: 'actuator_servos (out)', dir: 'outgoing', loc: 'actuators/' },
  { name: 'Hardpoint', msg: 'hardpoint.Command', topic: 'command (out)', dir: 'outgoing', loc: 'actuators/' },
]

const cfg = args || {}
const only = Array.isArray(cfg) ? cfg : (cfg.bridges || null)
const targets = only ? BRIDGES.filter(b => only.includes(b.name)) : BRIDGES

const extractPrompt = (b) =>
  'You are extracting the acceptance spec for one PX4 DroneCAN bridge, so a clean-room libcanard reimplementation can be verified field-for-field. PX4 root = ' + PX4 + '. Use Read/Grep/Bash and confirm every line number.\n\n' +
  'BRIDGE: ' + b.name + '\n' +
  'DroneCAN message(s): ' + b.msg + '\n' +
  'uORB topic: ' + b.topic + '  (direction: ' + b.dir + ')\n\n' +
  'Find the current libuavcan implementation under ' + PX4 + '/src/drivers/uavcan/' + b.loc + ' (grep for the message type and the topic; the file name may differ from the bridge name). Read it fully.\n' +
  'Extract the COMPLETE field-by-field mapping between the DroneCAN message(s) and the uORB topic: every uORB field, the message field/expression it derives from, and any unit conversion / scaling / remap (NED, rad<->deg, Pa<->mbar, etc). Also capture: the CONFIG_UAVCAN_* compile gate, the UAVCAN_SUB_*/PUB_* runtime param gate, redundancy / multi-instance handling (per-source-node channel allocation), and how the uORB device_id is encoded. ' +
  'For outgoing bridges, map the uORB-subscription fields to the outgoing DroneCAN message fields. This table is the TEST ORACLE used to verify the ported dronecan bridge: be exhaustive and exact, with path:line for the source. Fill the schema.'

phase('Extract')
log('P2 acceptance spec: extracting field maps for ' + targets.length + ' bridges/actuators')
const maps = (await parallel(targets.map(b => () =>
  agent(extractPrompt(b), { label: 'spec:' + b.name, phase: 'Extract', schema: FIELD_MAP_SCHEMA })
))).filter(Boolean)
log('Collected ' + maps.length + '/' + targets.length + ' field maps')

phase('Synthesize')
const outPath = PX4 + '/dronecan_migration/ACCEPTANCE_SPEC.md'
const synthPrompt =
  'You are the synthesis step of the DroneCAN acceptance-spec extraction. Below is structured JSON, one object per bridge/actuator, giving the exact current libuavcan message-to-uORB field mappings.\n\n' +
  'Write ONE markdown document to ' + outPath + ' using the Write tool (overwrite if present). It is the TEST ORACLE for porting: every ported dronecan bridge is verified against its table here.\n' +
  'Structure: (1) a 1-paragraph header explaining the oracle role and that field maps are normative (a ported bridge must reproduce them exactly, modulo the documented transforms); (2) one section per bridge with: source files (path:line), DroneCAN message(s), uORB topic + direction, a field-mapping TABLE (uORB field | source field/expr | transform), compile gate, runtime gate, redundancy, device-id encoding, and notes. Keep tables dense and exact. Preserve every path:line.\n' +
  'After writing, return a 3-line summary: the path, the bridge count, and any bridges where the mapping looked incomplete or ambiguous (so a human can spot-check).\n\n' +
  'STRUCTURED FIELD MAPS:\n' + JSON.stringify(maps, null, 2)

const result = await agent(synthPrompt, { label: 'synthesize -> ACCEPTANCE_SPEC.md', phase: 'Synthesize' })
log('P2 complete -> dronecan_migration/ACCEPTANCE_SPEC.md')
return result
