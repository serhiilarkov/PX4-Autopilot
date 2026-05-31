# DroneCAN Bridge Acceptance Spec (Test Oracle)

This document is the **test oracle** for the libuavcan-to-libcanard migration of the PX4 DroneCAN driver. It captures, per bridge/actuator, the exact DroneCAN-message-to-uORB field mappings as they exist in the current `src/drivers/uavcan` libuavcan implementation. Every ported bridge in `src/drivers/dronecan` is verified field-for-field against the table for that bridge here: **the field maps are normative**. A ported bridge MUST reproduce each `uORB field <- source field/expr` mapping *exactly*, including the documented transforms (unit conversions, sign flips, remaps, validity gates, fallthroughs, and "left at default" non-assignments), the compile gate, the runtime gate, the redundancy/channel-allocation model, and the device-id encoding. Where a current behavior is a latent bug or a quirk (FALLTHROUGH unpacking, microsecond-scaled leap-second math, single-shot `_inited` config, Kelvin-treated-as-Celsius offsets), it is called out explicitly and the port must replicate it byte-identically for parity unless the migration intentionally diverges. `path:line` references are preserved verbatim so a reviewer can diff the port against the exact source location. All paths are absolute under `/home/jake/code/jake/PX4-Autopilot`.

Bridges covered (18): GNSS, GNSS relative, Baro, Mag, Accel, Gyro, Airspeed, Differential pressure, Battery, Optical flow, Rangefinder, Hygrometer, Fuel tank, ICE status, Safety button, ESC, Servo, Hardpoint.

Common infrastructure (referenced by most bridges):
- **Channel allocation:** `UavcanSensorBridgeBase` keeps `_channels[_max_channels]`, `DEFAULT_MAX_CHANNELS = 4` (`src/drivers/uavcan/sensors/sensor_bridge.hpp:110`). Two publish paths exist: the simple `publish(node_id, report)` path (`sensor_bridge.cpp:256-317`, uses `orb_advertise_multi`) and the driver/CDev `get_channel_for_node(node_id, iface_index)` + `init_driver()` + `channel->h_driver` path (`sensor_bridge.cpp:319-374`). Most channel keys are **node_id only**; iface_index is stored at first allocation and used only for the device_id bus field. On exhaustion `_out_of_channels` latches and new nodes are dropped.
- **device_id helper:** `make_uavcan_device_id(node_id, iface_index)` (`sensor_bridge.hpp:161-169`) packs `device::Device::DeviceId` (`src/lib/drivers/device/Device.hpp:163-173`): `bus_type:3 = DeviceBusType_UAVCAN(3)`, `bus:5 = iface_index`, `address:8 = node_id`, `devtype:8 = get_device_type()`. Composite (LSB-first): `bits[0:2]=bus_type, [3:7]=bus, [8:15]=address, [16:23]=devtype`.
- **Temperature conversion:** Kelvin -> Celsius via `+ atmosphere::kAbsoluteNullCelsius` where `kAbsoluteNullCelsius = -273.15f` (`src/lib/atmosphere/atmosphere.h:54`).

---

## 1. GNSS

**Source files:**
- `src/drivers/uavcan/sensors/gnss.cpp:54-717`
- `src/drivers/uavcan/sensors/gnss.hpp:66-170`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:158-164`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:149-169`

**DroneCAN messages:**
- `uavcan.equipment.gnss.Fix` (1060, incoming, deprecated)
- `uavcan.equipment.gnss.Fix2` (1063, incoming, primary)
- `uavcan.equipment.gnss.Auxiliary` (1061, incoming, hdop/vdop)
- `ardupilot.gnss.RelPosHeading` (20006, incoming, heading)
- `ardupilot.gnss.MovingBaselineData` (20005, incoming -> gps_dump for PPK; AND outgoing when `UAVCAN_PUB_MBD=1`)
- `uavcan.equipment.gnss.RTCMStream` (1062, outgoing when `UAVCAN_PUB_RTCM=1`)

**uORB topic + direction:** `sensor_gps` (primary). Secondary outgoing: `gps_dump` (from incoming MovingBaselineData sub, `gnss.cpp:362-394`). Outgoing-path input: `gps_inject_data` (RTCM from GCS). **Bidirectional.**

**Field mapping (sensor_gps):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `hrt_absolute_time()` at publish (NOT msg time; FIXME hack `gnss.cpp:417-424`) | wall-clock us; `gnss.cpp:425` |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | devtype=0x85 (DRV_GPS_DEVTYPE_UAVCAN), address=node_id, bus_type=3, bus=iface. `gnss.cpp:408` |
| latitude_deg | `msg.latitude_deg_1e8` (int37) | `/1e8` (double). `gnss.cpp:427` |
| longitude_deg | `msg.longitude_deg_1e8` (int37) | `/1e8` (double). `gnss.cpp:428` |
| altitude_msl_m | `msg.height_msl_mm` (int27) | `/1e3` (mm->m). `gnss.cpp:429` |
| altitude_ellipsoid_m | `msg.height_ellipsoid_mm` (int27) | `/1e3` (mm->m). `gnss.cpp:430` |
| eph | `max(pos_cov[0], pos_cov[4])` if valid_pos_cov else -1 | `sqrt(h_var)` if >0 else -1.0; m. pos_cov is m^2. `gnss.cpp:432-443` |
| epv | `pos_cov[8]` if valid_pos_cov else -1 | `sqrt(pos_cov[8])` if >0 else -1.0; m. `gnss.cpp:438` |
| s_variance_m_s | `max(vel_cov[0],vel_cov[4],vel_cov[8])` if valid else -1 | direct (stored as variance). `gnss.cpp:446,468` |
| c_variance_rad | Jacobian of vel cov to heading var using `ned_velocity[0]=vel_n,[1]=vel_e`, `vel_cov[0,1,4]` | `(vel_e^2*vel_cov[0] - 2*vel_n*vel_e*vel_cov[1] + vel_n^2*vel_cov[4]) / (vel_n^2+vel_e^2)^2`; else -1.0. `gnss.cpp:458-469` |
| fix_type | Fix: `msg.status`. Fix2: `msg.status` remapped by `msg.mode`/`msg.sub_mode` | Fix2 remap (`gnss.cpp:192-211`): MODE_DGPS->4; MODE_RTK+RTK_FLOAT->5; MODE_RTK+RTK_FIXED->6; else =status (0..3). Fix passes status directly (`gnss.cpp:166`). `gnss.cpp:472` |
| vel_n_m_s | `msg.ned_velocity[0]` | direct, m/s. `gnss.cpp:474` |
| vel_e_m_s | `msg.ned_velocity[1]` | direct, m/s. `gnss.cpp:475` |
| vel_d_m_s | `msg.ned_velocity[2]` | direct, m/s. `gnss.cpp:476` |
| vel_m_s | `Vector3f(ned_velocity[0..2]).norm()` | euclidean norm, m/s. `gnss.cpp:477` |
| cog_rad | `atan2f(vel_e_m_s, vel_n_m_s)` | atan2(E,N), rad [-PI,PI]. `gnss.cpp:478` |
| vel_ned_valid | constant `true` | always true. `gnss.cpp:479` |
| timestamp_time_relative | constant `0` | `gnss.cpp:481` |
| time_utc_usec | `UtcTime(msg.gnss_timestamp).toUSec()` adjusted by `gnss_time_standard` + `num_leap_seconds` | UTC: as-is. GPS: `ts - num_leap_seconds + 9` (only if leap>0; raw us). TAI: `ts - num_leap_seconds - 10` (only if leap>0). NONE/default: 0. `gnss.cpp:483-506`. Side effect `gnss.cpp:509-521`: sets CLOCK_REALTIME once if utc!=0 && fix>=2D && !_system_clock_set |
| satellites_used | `msg.sats_used` (uint6) | direct. `gnss.cpp:523` |
| hdop | `Auxiliary.hdop` if Auxiliary seen <2s ago, else `msg.pdop` | direct float16. `gnss.cpp:525-534`; cached `gnss.cpp:151` |
| vdop | `Auxiliary.vdop` if Auxiliary <2s, else `msg.pdop` | direct float16. `gnss.cpp:525-534` |
| heading | RelPosHeading valid: `_rel_heading`; else Fix2: `ecef_position_velocity[0].velocity_xyz[0]` hack; Fix: NAN | RelPosHeading: `radians(reported_heading_deg)`, consumed once then reset NAN. `gnss.cpp:353-360,536-549`. Fix2 ECEF hack only if !empty && !rel_valid && !isnan (`gnss.cpp:329-332`). Fix=NAN (`gnss.cpp:177`) |
| heading_offset | RelPosHeading valid: NAN; else Fix2: `ecef_position_velocity[0].velocity_xyz[1]` hack; Fix: NAN | raw ECEF value, no scaling. `gnss.cpp:334-336,538,547` |
| heading_accuracy | RelPosHeading valid: `_rel_heading_accuracy`; else Fix2: `ecef_position_velocity[0].velocity_xyz[2]` hack; Fix: NAN | RelPosHeading: `radians(reported_heading_acc_deg)`. `gnss.cpp:358,539`; Fix2 raw `gnss.cpp:338-339,548` |
| noise_per_ms | Fix2: `ecef_position_velocity[0].position_xyz_mm[0]` (int36) hack; Fix: -1 | cast to int32 only when ECEF hack active. `gnss.cpp:342`; Fix -1 |
| jamming_indicator | Fix2: `ecef_position_velocity[0].position_xyz_mm[1]` hack; Fix: -1 | cast int32. `gnss.cpp:343` |
| jamming_state | Fix2: `position_xyz_mm[2] >> 8`; Fix: 0 | high byte. `gnss.cpp:345` |
| spoofing_state | Fix2: `position_xyz_mm[2] & 0xFF`; Fix: 0 | low byte. `gnss.cpp:346` |
| selected_rtcm_instance | `_selected_rtcm_instance` (gps_inject_data instance) | direct. `gnss.cpp:556`; selection `gnss.cpp:590-606` |
| rtcm_injection_rate | `_rtcm_injection_rate` (Hz over 5s window) | msgs/dt over 5s. `gnss.cpp:557,576-582` |
| **(OUT)** RTCMStream.protocol_id | constant `PROTOCOL_ID_RTCM3 (=3)` | `gnss.cpp:656` |
| **(OUT)** RTCMStream.data | `gps_inject_data_s.data/.len` | chunked <=128 B (`msg.data.capacity()`), priority NumericallyMax. `gnss.cpp:652-680`. Gate `UAVCAN_PUB_RTCM` |
| **(OUT)** MovingBaselineData.data | `gps_inject_data_s.data/.len` | chunked <=300 B, priority NumericallyMax. `gnss.cpp:682-708`. Gate `UAVCAN_PUB_MBD` |
| **(SEC OUT)** gps_dump.* (from incoming MBD sub) | `MovingBaselineData.data -> gps_dump.data`; `getSrcNodeID -> device_id`; len; instance=0 | chunked into `sizeof(dump.data)`; device_id hand-built (bus_type=UAVCAN, bus=0, address=node_id&0xFF, devtype=0x85); `dump.len &= 0x7F`; instance hardcoded 0. `gnss.cpp:362-394`. Gate `UAVCAN_SUB_MBD` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_GNSS` (`Kconfig:65-67`, default y). Guarded at `sensor_bridge.cpp:63-65` and `:158-166`. Separate `CONFIG_UAVCAN_SENSOR_GNSS_RELATIVE` (`Kconfig:69-71`) gates the distinct UavcanGnssRelativeBridge (`sensor_gnss_relative`, see section 2) — shares the RelPosHeading message but is out of scope for this bridge.

**Runtime gate:** `UAVCAN_SUB_GPS` (default 1, reboot_required) gates whole bridge (`sensor_bridge.cpp:159-164`; `uavcan_params.yaml:213-223`). Sub-gates in `init()`: `UAVCAN_PUB_RTCM` (default 0, yaml:130) -> RTCMStream pub, priority NumericallyMax (`gnss.cpp:114-122`); `UAVCAN_PUB_MBD` (default 0, yaml:139) -> MBD pub (`gnss.cpp:124-132`); `UAVCAN_SUB_MBD` (default 0, yaml:288) -> incoming MBD->gps_dump sub (`gnss.cpp:134-141`). Auxiliary/Fix/Fix2/RelPosHeading subscribers always started (`gnss.cpp:86-112`). Whole stack gated by `UAVCAN_ENABLE` (yaml:5).

**Redundancy:** Per-source-node channel alloc in `publish` (`sensor_bridge.cpp:256-317`), up to 4 channels keyed by node_id -> distinct `sensor_gps` instance. `get_channel_index_for_node` (`sensor_bridge.cpp:389-401`) used by Fix/Fix2. **Fix-vs-Fix2 dedup per node:** `_channel_using_fix2[ch]` (`gnss.hpp:154`, alloc `gnss.cpp:66-70`); a node sending Fix2 sets flag (`gnss.cpp:185-190`) and subsequent Fix from that node are ignored (`gnss.cpp:160-164`). Flag latches only once a channel exists (get_channel_index_for_node returns -1 before publish runs). **Auxiliary hdop/vdop and RelPosHeading heading are GLOBAL** (single member vars, last-writer-wins across nodes, `gnss.cpp:142-144,159-161`). RTCM source selection per gps_inject_data instance, independent of node redundancy (`gnss.cpp:590-606`).

**Device-id encoding:** `make_uavcan_device_id(node_id, iface_index)` -> `{bus_type:3=UAVCAN(3); bus:5=iface_index(0=CAN1,1=CAN2); address:8=node_id; devtype:8=DRV_GPS_DEVTYPE_UAVCAN=0x85}` (`sensor_bridge.hpp:161-169`; set via `set_device_type` `gnss.cpp:72`; `drv_sensor.h:187`). Base class separately overwrites `_device_id.address/bus_type` on channel creation (`sensor_bridge.cpp:294-296`) but that internal id is NOT written to the report — each `sensor_gps.device_id` comes from `make_uavcan_device_id`. **Divergence:** `gps_dump.device_id` (incoming MBD path) hand-built with `bus=0` (NOT iface) and `address=node_id&0xFF` (`gnss.cpp:374-378`).

**Notes:**
- Topic confusion guard: `sensor_gnss_relative` is produced by the SEPARATE bridge (`gnss_relative.cpp:42`), NOT here. This bridge uses RelPosHeading only for sensor_gps heading/heading_accuracy (`gnss.cpp:353-360`). Both bridges subscribe RelPosHeading independently.
- **Fix2 covariance unpacking** (`gnss.cpp:213-315`) handles 4 sizes of `msg.covariance` (float16[<=36]): size1=scalar (diag of pos+vel); size6=diagonal; size21=upper-tri 6x6; size36=full 6x6; else valid_covariances=false. **Cases 21 and 36 have intentional `/* FALLTHROUGH */`** so a size-21 msg also runs the size-36 block (overwriting pos/vel cov) — a clean-room port MUST replicate this fallthrough though it appears a latent bug.
- Fix (deprecated) path unpacks position_covariance/velocity_covariance (float16[<=9]) via `unpackSquareMatrix` before process; validity = `!empty()` (`gnss.cpp:168-177`). Fix passes heading=NAN, noise/jamming/spoofing = -1/-1/0/0.
- **UTC subtlety:** GPS/TAI leap correction (`-num_leap_seconds +9 / -10`) is applied in MICROSECONDS to a microsecond timestamp (`gnss.cpp:492,499`) — a tiny/incorrect offset vs the seconds-scaled DSDL formula. Replicate verbatim. Leap adjust only when `num_leap_seconds>0`; if UNKNOWN(0), GPS/TAI leave utc=0.
- `ned_velocity` type differs: Fix float16[3], Fix2 float32[3] (DSDL 1060 vs 1063). Both [0]=N,[1]=E,[2]=D.
- Auxiliary freshness window 2s (`gnss.cpp:525`). Fallback uses `msg.pdop` for BOTH hdop and vdop (issue #5153).
- `_system_clock_set` sets CLOCK_REALTIME exactly once across bridge lifetime (`gnss.cpp:509-521`).
- RTCM chunk sizes from `msg.data.capacity()`: RTCMStream=128, MBD=300; gps_dump chunk=`sizeof(dump.data)`.
- `handleInjectDataTopic` (`gnss.cpp:572-650`) drives outgoing RTCM/MBD from gps_inject_data: link-failover after 5s stale (`gnss.cpp:590-606`), max 8/cycle, updates rate every 5s. Runs from `update()` (`gnss.cpp:562-565`).
- `NodeInfoPublisher::registerDeviceCapability(node_id, device_id, GPS)` on first msg/node (`gnss.cpp:411-415`) — side effect, not a uORB field.

---

## 2. GNSS relative (UavcanGnssRelativeBridge)

**Source files:**
- `src/drivers/uavcan/sensors/gnss_relative.cpp:39-86`
- `src/drivers/uavcan/sensors/gnss_relative.hpp:43-64`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:66-68` (include gate)
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:168-177` (registration + runtime gate)
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317` (publish/channel alloc)
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:90-97,114-125,149-169`
- `src/drivers/uavcan/CMakeLists.txt:176`
- `src/drivers/uavcan/Kconfig:69-71`
- `src/drivers/uavcan/uavcan_params.yaml:224-232`
- `src/drivers/uavcan/libdronecan/dsdl/ardupilot/gnss/20006.RelPosHeading.uavcan:1-9`
- `msg/SensorGnssRelative.msg:1-31`
- `src/drivers/drv_sensor.h:187`
- `src/lib/drivers/device/Device.hpp:147-172`

**DroneCAN message:** `ardupilot.gnss.RelPosHeading` (DTID 20006, sig 0xB2F757F09F08BCD0). Fields: `Timestamp timestamp; bool reported_heading_acc_available; float32 reported_heading_deg; float32 reported_heading_acc_deg; float16 relative_distance_m; float16 relative_down_pos_m`.

**uORB topic + direction:** `sensor_gnss_relative`, **incoming**.

**Field mapping (sensor_gnss_relative):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp_sample | `msg.timestamp` | `UtcTime(msg.timestamp).toUSec()` -> us. `gnss_relative.cpp:66` |
| heading_valid | `msg.reported_heading_acc_available` | direct bool. `gnss_relative.cpp:68` |
| heading | `msg.reported_heading_deg` (deg) | `radians(...)` deg->rad. `gnss_relative.cpp:69` |
| heading_accuracy | `msg.reported_heading_acc_deg` (deg) | `radians(...)` deg->rad. `gnss_relative.cpp:70` |
| position_length | `msg.relative_distance_m` (f16) | direct (f16->f32), m. `gnss_relative.cpp:71` |
| position[2] | `msg.relative_down_pos_m` (f16) | direct (f16->f32) -> NED Down; position[0]/[1] left 0. `gnss_relative.cpp:72` |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | encoded (see below). `gnss_relative.cpp:74` |
| timestamp | `hrt_absolute_time()` | publish time us. `gnss_relative.cpp:83` |
| time_utc_usec | (none) | NOT SET -> 0. The only real-UTC field, left 0; msg time goes to timestamp_sample. `gnss_relative.cpp:64` |
| reference_station_id | (none) | 0 |
| position[0] / position[1] | (none) | 0 (N/E left zero; msg has no N/E) |
| position_accuracy[0..2] | (none) | 0,0,0 |
| accuracy_length | (none) | 0 |
| gnss_fix_ok, differential_solution, relative_position_valid, carrier_solution_floating, carrier_solution_fixed, moving_base_mode, reference_position_miss, reference_observations_miss, relative_position_normalized | (none) | all NOT SET -> false |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_GNSS_RELATIVE` (`Kconfig:69-71`, default y). Guards `#include` (`sensor_bridge.cpp:66-68`) and registration (`sensor_bridge.cpp:169-177`). `gnss_relative.cpp` always compiled (`CMakeLists.txt:176`); bridge only instantiated when symbol defined.

**Runtime gate:** `UAVCAN_SUB_GPS_R` (bool, default 1, reboot_required; `uavcan_params.yaml:224-232`). Read `sensor_bridge.cpp:170-171`; added only if != 0 (`sensor_bridge.cpp:173-175`).

**Redundancy:** Per-source-node multi-instance, up to 4 channels (`sensor_bridge.hpp:110`, default ctor `gnss_relative.cpp:42`). `publish()` keyed by node_id (`gnss_relative.cpp:85`) -> `orb_advertise_multi(ORB_ID(sensor_gnss_relative))` per node. Key is node_id ONLY (different iface from same node -> same channel). On exhaustion/advertise-fail `_out_of_channels` latches. device_id is per-message (`make_uavcan_device_id`, includes iface) not per-channel; base `_device_id` overwrite at `sensor_bridge.cpp:295-296` is NOT what gets published.

**Device-id encoding:** `make_uavcan_device_id(node_id, iface_index)`: devtype=DRV_GPS_DEVTYPE_UAVCAN=0x85 (ctor `gnss_relative.cpp:45`); address=node_id; bus_type=UAVCAN(3); bus=iface_index. Example: node 125 on CAN1 -> `0x00857D03`.

**Notes:**
- **INCOMPLETE MAPPING IS INTENTIONAL:** only 5 msg fields + timestamp exist; struct zero-init (`gnss_relative.cpp:64`); only the 8 listed fields populated. A faithful port MUST leave all other uORB fields 0/false.
- Unit conversions: heading/heading_acc deg->rad via `math::radians()`; relative_distance/down_pos f16->f32 no scale. No NED rotation/sign change; only Down filled, N/E zero.
- Timestamp semantics: msg UTC us -> `timestamp_sample`, NOT `time_utc_usec`. `timestamp` = hrt at publish.
- DSDL note: tree type is `ardupilot.gnss.RelPosHeading` (DTID 20006); Kconfig/param help text mislabels it `ardupilot::equipment::gnss::RelPosHeading` but actual C++ type is `ardupilot::gnss::RelPosHeading` (`gnss_relative.hpp:41,56,62`).
- NodeInfoPublisher side effect: registers GPS DeviceCapability `(srcNodeId, device_id, GPS)` after first msg (`gnss_relative.cpp:76-81`), guarded non-null.
- Base name passed to Device = `uavcan_gnss_relative` (`gnss_relative.cpp:42`); `get_name()` = `gnss_relative`. `set_device_bus(0)` in base ctor but bus is overwritten with iface_index by make_uavcan_device_id.

---

## 3. Baro

**Source files:**
- `src/drivers/uavcan/sensors/baro.cpp:46-217`
- `src/drivers/uavcan/sensors/baro.hpp:45-85`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:102-111`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:319-374`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:149-169`

**DroneCAN messages:** `uavcan.equipment.air_data.StaticPressure` (1028); `StaticTemperature` (1029); `RawAirData` (1027).

**uORB topic + direction:** `sensor_baro`, **incoming**.

**Field mapping (sensor_baro):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp_sample | `hrt_absolute_time()` at callback entry | direct (us since boot), sampled at handler entry. `baro.cpp:102` (StaticPressure), `:153` (RawAirData) |
| device_id | `make_uavcan_device_id(msg)` | devtype=0x81 (DRV_BARO_DEVTYPE_UAVCAN), address=node_id, bus_type=UAVCAN, bus=iface. `baro.cpp:118`/`:169`. StaticTemperature cb builds NO device_id |
| pressure | `StaticPressure.static_pressure` (`baro.cpp:130`) OR `RawAirData.static_pressure` (`baro.cpp:181`) | direct Pa->Pa, no scaling |
| temperature | StaticPressure path: cached `_last_temperature_kelvin` (from StaticTemperature). RawAirData path: `RawAirData.static_air_temperature` | Kelvin->degC: `value + kAbsoluteNullCelsius` if finite && >=0K else NAN. SP `baro.cpp:132-137`; RAD `baro.cpp:185-190` |
| error_count | constant 0 | hardcoded. `baro.cpp:139`/`:192` |
| timestamp | `hrt_absolute_time()` again at publish | direct (us); later read than timestamp_sample. `baro.cpp:140`/`:193` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_BARO` (`Kconfig:45-47`, default y). Guards include (`sensor_bridge.cpp:48-50`) and instantiation (`sensor_bridge.cpp:102-111`).

**Runtime gate:** `UAVCAN_SUB_BARO` (`uavcan_params.yaml:159-168`, bool, default 0, reboot_required). Read `sensor_bridge.cpp:104-105`; added if != 0. **NOTE:** C++ local fallback is 1 (`line 104`) but registered param default is 0, so effective out-of-box = DISABLED.

**Redundancy:** `get_channel_for_node(node_id, iface_index)` (`sensor_bridge.cpp:319-374`), 4 channels. Key is **node_id ONLY** (`lines 324-329,341-346`); iface stored at first alloc (`line 357`) for device_id bus only -> node on multiple ifaces maps to ONE channel (first iface). First new node -> `init_driver(channel)` (`baro.cpp:197-217`) creates `uORB::PublicationMulti<sensor_baro_s>(ORB_ID(sensor_baro))` in `channel->h_driver`, `channel->instance = baro->get_instance()`. On fail/exhaustion `_out_of_channels` latches (`333-336,349-353,360-367`). Uses the `h_driver`/PublicationMulti path, NOT base `publish()`.

**Device-id encoding:** `make_uavcan_device_id(msg)`: devtype=DRV_BARO_DEVTYPE_UAVCAN=0x81 (ctor `baro.cpp:54`; `drv_sensor.h:183`); address=node_id; bus_type=UAVCAN; bus=iface_index. Written per-message at `baro.cpp:118`/`:169`.

**Notes:**
- THREE subscribers, ONE publication path. StaticPressure cb (`baro.cpp:99-142`) and RawAirData cb (`baro.cpp:144-195`) each build+publish full `sensor_baro_s`. StaticTemperature cb (`baro.cpp:83-97`) publishes NOTHING — only updates `_last_temperature_kelvin` (init NAN, `baro.hpp:83`), consumed by the StaticPressure path. RawAirData does NOT use the cache (uses its own static_air_temperature).
- **StaticTemperature legacy-compat double-shift** (`baro.cpp:86-96`, PR #19061): if `static_temperature >= 0` take as Kelvin directly. If `< 0`: `temperature_c = static_temperature - kAbsoluteNullCelsius` (i.e. +273.15); then ONLY IF `-40 < temperature_c < 120`, `_last_temperature_kelvin = temperature_c - kAbsoluteNullCelsius` (+273.15 again). Outside that range cache left unchanged. Port must reproduce this exact double-shift.
- **RawAirData input gate** (`baro.cpp:147-151`): returns early unless `PX4_ISFINITE(msg.static_pressure) && msg.static_pressure > 0.0f`. So 0/NaN static_pressure -> no publish from RawAirData.
- StaticPressure path has NO validity gate — always publishes whatever static_pressure (even 0/NaN) once channel exists.
- NodeInfoPublisher: registerDeviceCapability(srcNodeID, device_id, BAROMETER) on first StaticPressure (`baro.cpp:121-124`) or RawAirData (`baro.cpp:172-175`); NOT in StaticTemperature cb.
- `sensor_baro` has NO variance field; DSDL `*_variance`/covariance/differential_pressure/`*_sensor_temperature`/pitot_temperature are dropped.
- Publisher `uORB::PublicationMulti<sensor_baro_s>` lazily per node in `init_driver`; NAME=`baro` (`baro.cpp:46`), base name `uavcan_baro` (`:49`).
- DSDL units: SP.static_pressure=Pa(f32); ST.static_temperature=K(f16); RAD.static_pressure=Pa(f32), RAD.static_air_temperature=K(f16).

---

## 4. Mag

**Source files:**
- `src/drivers/uavcan/sensors/mag.cpp:45-159`
- `src/drivers/uavcan/sensors/mag.hpp:42-75`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:214-222`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:319-374`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:161-169`
- `src/lib/drivers/magnetometer/PX4Magnetometer.cpp:63-80`

**DroneCAN messages:** `uavcan.equipment.ahrs.MagneticFieldStrength` (1001, DEPRECATED); `MagneticFieldStrength2` (1002).

**uORB topic + direction:** `sensor_mag`, **incoming**.

**Field mapping (sensor_mag):**

| uORB field | source field/expr | transform |
|---|---|---|
| x | `magnetic_field_ga[0]` (both msgs) | direct copy * _scale(=1.0), ROTATION_NONE no-op; Gauss->Gauss, no NED remap. `mag.cpp:97,129` -> `PX4Magnetometer.cpp:72,74` |
| y | `magnetic_field_ga[1]` | direct copy * _scale(=1.0). `mag.cpp:98,130` -> `PX4Magnetometer.cpp:75` |
| z | `magnetic_field_ga[2]` | direct copy * _scale(=1.0). `mag.cpp:99,131` -> `PX4Magnetometer.cpp:76` |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | {bus_type=3, bus=iface, address=node_id, devtype=0x88 DRV_MAG_DEVTYPE_UAVCAN}; computed at init_driver (`mag.cpp:139`), -> report (`PX4Magnetometer.cpp:67`) |
| timestamp_sample | `hrt_absolute_time()` at callback dispatch | passed to `mag->update(hrt,x,y,z)`. `mag.cpp:101,133` -> `PX4Magnetometer.cpp:66`. NOT from CAN timestamp |
| timestamp | `hrt_absolute_time()` at publish | `PX4Magnetometer.cpp:78` |
| temperature | constant NAN | `_temperature` default NAN (`PX4Magnetometer.hpp:65`); never updated. `PX4Magnetometer.cpp:68` |
| error_count | constant 0 | `_error_count` default 0; never updated. `PX4Magnetometer.cpp:69` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_MAG` (`Kconfig:81-83`, default y). Gates `#include` (`sensor_bridge.cpp:75-77`) and instantiation (`sensor_bridge.cpp:214-222`).

**Runtime gate:** `UAVCAN_SUB_MAG` (`uavcan_params.yaml:260-269`, bool, default 1, reboot_required). `sensor_bridge.cpp:215-219`: if != 0 add bridge. Default 1 = subscribed.

**Redundancy:** Per-source-node channel alloc, 4 channels. `get_channel_for_node(node_id, iface_index)` (`mag.cpp:77,108`); first new node -> `init_driver()` constructs dedicated `PX4Magnetometer` -> own `sensor_mag` instance via PublicationMulti (`orb_advertise_multi`), `channel->instance = mag->get_instance()` (`mag.cpp:149`). Both v1+v2 route through same channel set keyed on **node_id**. On fail `_out_of_channels` latches (`sensor_bridge.cpp:288-291,360-368`). **NOTE:** v2 cb additionally rejects when `channel->instance < 0` (`mag.cpp:110`); v1 cb only null-checks channel (`mag.cpp:79`). iface recorded into channel/device_id on first alloc only.

**Device-id encoding:** `make_uavcan_device_id(node_id, iface_index)`: bus_type=UAVCAN(3); bus=iface; address=node_id; devtype=0x88 (DRV_MAG_DEVTYPE_UAVCAN, `drv_sensor.h:190`). `set_device_type` in ctor `mag.cpp:52`.

**Notes:**
- Subscribes BOTH 1001 (DEPRECATED) and 1002. Both started in `init()` (`mag.cpp:57,64`); either `start()` failure aborts init.
- DSDL: 1001 `float16[3] magnetic_field_ga; float16[<=9] covariance`. 1002 `uint8 sensor_id; float16[3] magnetic_field_ga; float16[<=9] covariance`. Units GAUSS body frame (float16). Decoder must read float16 arrays.
- UNUSED (decode for wire correctness, drop): `magnetic_field_covariance` (both) never read; `sensor_id` (v2) never read and does NOT affect channel/instance (redundancy keyed solely on node_id + iface, NOT sensor_id).
- No coordinate remap, no scaling: msg Gauss(FRD) -> sensor_mag Gauss(FRD). `_scale` stays 1.0 (set_scale never called), `_rotation` ROTATION_NONE (`mag.cpp:141`), rotate_3f no-op. Port must NOT add rad/deg, NED/FRD, or Tesla/Gauss conversion.
- temperature always NAN, error_count always 0 (set_temperature/set_error_count never called).
- NodeInfoPublisher side-effect: registerDeviceCapability(srcNodeID, device_id, MAGNETOMETER) per msg (`mag.cpp:92-95,123-127`).
- `sensor_mag` ORB_QUEUE_LENGTH=4 (`SensorMag.msg:14`).

---

## 5. Accel (UavcanAccelBridge)

**Source files:**
- `src/drivers/uavcan/sensors/accel.cpp:41,43,47,62,64,66,80,81,90,93,95,103`
- `src/drivers/uavcan/sensors/accel.hpp:41,65`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:41,202,203,204,206,207,319`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:110,116,161`
- `src/lib/drivers/accelerometer/PX4Accelerometer.cpp:114`
- `src/lib/drivers/accelerometer/PX4Accelerometer.hpp:69,77,78`
- `src/drivers/uavcan/uavcan_params.yaml:251`
- `src/drivers/uavcan/Kconfig:37`
- `src/drivers/drv_sensor.h:182`
- `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/ahrs/1003.RawIMU.uavcan:36`

**DroneCAN message:** `uavcan.equipment.ahrs.RawIMU` (DTID 1003).

**uORB topic + direction:** `sensor_accel`, **incoming**.

**Field mapping (sensor_accel):**

| uORB field | source field/expr | transform |
|---|---|---|
| x | `msg.accelerometer_latest[0]` | rotate_3f(ROTATION_NONE) then *_scale(1.0) = pass-through; m/s^2 (X fwd). f16->f32. `PX4Accelerometer.cpp:117,126` |
| y | `msg.accelerometer_latest[1]` | pass-through; m/s^2 (Y right). `PX4Accelerometer.cpp:117,127` |
| z | `msg.accelerometer_latest[2]` | pass-through; m/s^2 (Z down). `PX4Accelerometer.cpp:117,128` |
| timestamp_sample | `msg.timestamp.usec` | `(usec>0) ? usec : hrt_absolute_time()`. `accel.cpp:66` -> `PX4Accelerometer.cpp:122` |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | encoded (see below); set init_driver (`accel.cpp:93,95`) -> `PX4Accelerometer.cpp:123` |
| error_count | constant 0 | `set_error_count(0)` on EVERY msg (`accel.cpp:80`); always 0. `PX4Accelerometer.cpp:125` |
| temperature | not provided | `_temperature` default NaN (`PX4Accelerometer.hpp:79`); always NaN. `PX4Accelerometer.cpp:124` |
| clip_counter[0] | rotated x vs `_clip_limit` | `(fabsf(x) >= _clip_limit)`; `_clip_limit = fabsf(_range/_scale*0.999f) = 16*9.80665*0.999 ~=156.7` (_range=16*ONE_G, _scale=1). `PX4Accelerometer.cpp:129,182` |
| clip_counter[1] | rotated y | `(fabsf(y) >= _clip_limit)`. `PX4Accelerometer.cpp:130` |
| clip_counter[2] | rotated z | `(fabsf(z) >= _clip_limit)`. `PX4Accelerometer.cpp:131` |
| samples | constant 1 | non-FIFO path. `PX4Accelerometer.cpp:132` |
| timestamp | `hrt_absolute_time()` | publish time (NOT sample time). `PX4Accelerometer.cpp:133` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_ACCEL` (`Kconfig:37`, default y). Guards `accel.hpp` include (`sensor_bridge.cpp:41`) and registration (`sensor_bridge.cpp:202-211`).

**Runtime gate:** `UAVCAN_SUB_IMU` (`uavcan_params.yaml:251`, bool, default 0, reboot_required). Read `sensor_bridge.cpp:203-204`; if != 0 BOTH UavcanAccelBridge AND UavcanGyroBridge added (`sensor_bridge.cpp:206-209`) — single param gates accel+gyro from one RawIMU sub.

**Redundancy:** `get_channel_for_node(node_id, iface_index)` (`sensor_bridge.cpp:319`), 4 channels. First RawIMU from a node -> `init_driver()` creates dedicated `PX4Accelerometer` -> one `sensor_accel` instance per node (`channel->instance = accel->get_instance()`, `accel.cpp:103`; PublicationMulti `PX4Accelerometer.hpp:69`). Lookup keyed on **node_id only** (iface stored not matched, `sensor_bridge.cpp:325`). Exhaustion -> `_out_of_channels` (`333,349-352`). Callback selects channel by (node_id, iface) `accel.cpp:64`.

**Device-id encoding:** `make_uavcan_device_id(node_id, iface_index)`: devtype=DRV_ACC_DEVTYPE_UAVCAN=0x80 (ctor `accel.cpp:47`; `drv_sensor.h:182`); address=node_id; bus_type=UAVCAN; bus=iface. Built once init_driver (`accel.cpp:93`) -> PX4Accelerometer ctor (`accel.cpp:95`). bus is fixed at iface of FIRST msg that allocated the channel.

**Notes:**
- Strictly incoming. Gyro half (rate_gyro_latest) handled by separate UavcanGyroBridge, same UAVCAN_SUB_IMU gate.
- Only `accelerometer_latest[0..2]` (f16, m/s^2) consumed. `accelerometer_integral`, `rate_gyro_latest`, `rate_gyro_integral`, `integration_interval`, covariance NOT read.
- No unit conversion: _scale=1.0 (`PX4Accelerometer.hpp:78`), _rotation=ROTATION_NONE (single-arg ctor `accel.cpp:95`). RawIMU accel axes (X fwd, Y right, Z down) match PX4 body; no NED/axis remap.
- error_count hard-forced 0 each msg. temperature NaN. timestamp(publish) != timestamp_sample(bus or hrt fallback).
- clip_counter limit ~156.74 m/s^2 (ONE_G=9.80665); compared on rotated pre-scale value (scale=1 so identical). samples always 1.
- RawIMU DefaultDataTypeID=1003 (RawIMU.hpp:190-191).
- NodeInfoPublisher: registers ACCELEROMETER per msg if non-null (`accel.cpp:84-87`).
- PX4Accelerometer advertises immediately in ctor (`PX4Accelerometer.cpp:74`); get_instance() must be >=0 or init_driver fails (`accel.cpp:105-110`).

---

## 6. Gyro

**Source files:**
- `src/drivers/uavcan/sensors/gyro.cpp:43-114`
- `src/drivers/uavcan/sensors/gyro.hpp:42-67`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:201-211`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:319-374`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:90-169`
- `src/lib/drivers/gyroscope/PX4Gyroscope.cpp:69-135`
- `src/lib/drivers/gyroscope/PX4Gyroscope.hpp:53-79`

**DroneCAN message:** `uavcan.equipment.ahrs.RawIMU` (DTID 1003).

**uORB topic + direction:** `sensor_gyro`, **incoming**.

**Field mapping (sensor_gyro):**

| uORB field | source field/expr | transform |
|---|---|---|
| x | `msg.rate_gyro_latest[0]` (f16, rad/s, roll/X) | `report.x = x * _scale(1.f)`; ROTATION_NONE; direct passthrough rad/s |
| y | `msg.rate_gyro_latest[1]` (pitch/Y) | `* _scale(1.f)`; passthrough rad/s |
| z | `msg.rate_gyro_latest[2]` (yaw/Z) | `* _scale(1.f)`; passthrough rad/s |
| timestamp_sample | `msg.timestamp.usec` | `(usec>0) ? usec : hrt_absolute_time()`. `gyro.cpp:78` -> `PX4Gyroscope.cpp:121` |
| timestamp | `hrt_absolute_time()` | publish time. `PX4Gyroscope.cpp:132` |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | devtype overlaid 0x86 by set_device_type ctor (`gyro.cpp:47`); built init_driver `gyro.cpp:94`. `PX4Gyroscope.cpp:122` |
| temperature | not provided | `_temperature` default NAN; never set -> NAN. `PX4Gyroscope.cpp:123` |
| error_count | not set | `_error_count` default 0. `PX4Gyroscope.cpp:124` |
| clip_counter[0] | x vs `_clip_limit` | `(fabsf(x) >= _clip_limit)`; `_clip_limit = _range/_scale`. `PX4Gyroscope.cpp:128` |
| clip_counter[1] | y | `(fabsf(y) >= _clip_limit)`. `PX4Gyroscope.cpp:129` |
| clip_counter[2] | z | `(fabsf(z) >= _clip_limit)`. `PX4Gyroscope.cpp:130` |
| samples | constant 1 | non-FIFO single-sample. `PX4Gyroscope.cpp:131` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_ACCEL` (`Kconfig:37-39`, default y). **NO separate CONFIG_UAVCAN_SENSOR_GYRO** — gyro gated by the same ACCEL/IMU symbol, #included and instantiated alongside accel (`sensor_bridge.cpp:41-44`, `:202-209`).

**Runtime gate:** `UAVCAN_SUB_IMU` (`sensor_bridge.cpp:203-209`). Code default 1, but `uavcan_params.yaml:251-259` default 0 (DISABLED; reboot_required). When non-zero, BOTH UavcanAccelBridge AND UavcanGyroBridge instantiated from one RawIMU.

**Redundancy:** Per-source-node, 4 channels. `imu_sub_cb` -> `get_channel_for_node(node_id, iface)` (`gyro.cpp:64`). First new node -> `init_driver()` new's `PX4Gyroscope`, `channel->instance = gyro->get_instance()`; PublicationMulti -> one `sensor_gyro` instance per node. `_out_of_channels` once 4 exhausted or alloc fails. Keyed by **node_id** only; iface of FIRST packet baked into device_id bus.

**Device-id encoding:** init_driver (`gyro.cpp:94`) `make_uavcan_device_id(node_id, iface_index)`: devtype=DRV_GYR_DEVTYPE_UAVCAN=0x86 (`drv_sensor.h:188`, set ctor `gyro.cpp:47`); address=node_id; bus_type=UAVCAN(3); bus=iface. -> PX4Gyroscope ctor (`PX4Gyroscope.cpp:70`). Base `publish()` path (`sensor_bridge.cpp:294-296`) NOT used by this bridge.

**Notes:**
- NAME=`gyro`; base name `uavcan_gyro` (`gyro.cpp:41,44`).
- Only `rate_gyro_latest[0..2]` consumed. ALL other RawIMU fields IGNORED (integration_interval, rate_gyro_integral, accelerometer_latest -> accel bridge, accelerometer_integral, covariance). Port reads only timestamp.usec and rate_gyro_latest.
- rate_gyro_latest float16 on wire; apply IEEE-754 half->float.
- NO unit conversion: rad/s -> rad/s, _scale=1.f (`PX4Gyroscope.hpp:76`). No NED/rad-deg/Pa-mbar.
- Rotation: single-arg ctor (`gyro.cpp:96`), ROTATION_NONE default, rotate_3f no-op; axes verbatim.
- Timestamp fallback: usec==0 -> hrt for timestamp_sample. timestamp always hrt at publish.
- FIFO-less path: samples=1, publishes sensor_gyro (not sensor_gyro_fifo).
- NodeInfoPublisher: registerDeviceCapability(srcNodeID, gyro->get_device_id(), GYROSCOPE) per cb (`gyro.cpp:85-88`).
- PX4Gyroscope advertises in ctor (`PX4Gyroscope.cpp:74`); get_instance() must be >=0 (`gyro.cpp:104-111`).

---

## 7. Airspeed

**Source files:**
- `src/drivers/uavcan/sensors/airspeed.cpp:43-116`
- `src/drivers/uavcan/sensors/airspeed.hpp:46-85`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:92-100`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:103-179`

**DroneCAN messages:**
- `uavcan.equipment.air_data.IndicatedAirspeed` (1021): float16 indicated_airspeed [m/s], float16 indicated_airspeed_variance
- `uavcan.equipment.air_data.TrueAirspeed` (1020): float16 true_airspeed [m/s], float16 true_airspeed_variance
- `uavcan.equipment.air_data.StaticTemperature` (1029): float16 static_temperature [K], float16 static_temperature_variance

**uORB topic + direction:** `airspeed`, **incoming**.

**Field mapping (airspeed):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `hrt_absolute_time()` (NOT CAN time) | direct; `airspeed.cpp:104`. Comment: getMonotonicTimestamp abandoned (TIM5 clock) |
| indicated_airspeed_m_s | `IndicatedAirspeed.indicated_airspeed` (f16) | direct copy, f16->f32 widen, no scale. `airspeed.cpp:105` |
| true_airspeed_m_s | `TrueAirspeed.true_airspeed` (f16), cached `_last_tas_m_s` | direct copy of last cached TAS, f16->f32. Cached `airspeed.cpp:89`, consumed `:106`. Defaults 0.0f if no TAS ever (`airspeed.hpp:82`) |
| timestamp_sample | NONE | left 0 (report{} zero-init `airspeed.cpp:95`) |
| confidence | NONE | left 0. `*_variance` NOT mapped to confidence; variances ignored entirely |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_AIRSPEED` (`Kconfig:41-43`, bool, default y). Gates `#include` (`sensor_bridge.cpp:45-47`) and construction (`sensor_bridge.cpp:92-100`).

**Runtime gate:** `UAVCAN_SUB_ASPD` (`uavcan_params.yaml:148-158`, bool, default 0, reboot_required). Read `sensor_bridge.cpp:93-94`; code fallback 1, **actual param default 0 (OFF)**. Added if != 0 (`sensor_bridge.cpp:96-98`).

**Redundancy:** Per-source-node via base `publish()` (`sensor_bridge.cpp:256-317`). Each SrcNodeID (`airspeed.cpp:108`) gets own Channel + own uORB instance via `orb_advertise_multi` (`sensor_bridge.cpp:299`). Capacity 4 (`sensor_bridge.hpp:110-111`). Exhaustion/advertise-fail -> `_out_of_channels`, new reports dropped. **Publish trigger is IndicatedAirspeed ONLY**; TrueAirspeed/StaticTemperature do not create channels or publish.

**Device-id encoding:** Does NOT use `make_uavcan_device_id()`; `airspeed` topic has NO device_id field, so none is ever encoded into the message. The legacy `publish()` mutates base `_device_id` (`sensor_bridge.cpp:295-296`: address=node_id, bus_type=UAVCAN) but never writes it to the report. Resulting (unpublished) `_device_id`: devtype=0 (set_device_type never called), bus=0 (`sensor_bridge.hpp:124`), bus_type=UAVCAN, address=node_id. iface NOT encoded. Instance disambiguation via orb_advertise_multi index.

**Notes:**
- Publish edge-triggered EXCLUSIVELY by IndicatedAirspeed cb (`airspeed.cpp:91-115`). TrueAirspeed cb (`:85-90`) and StaticTemperature cb (`:78-83`) ONLY cache; never publish. One airspeed publication per IndicatedAirspeed frame carrying most-recent cached TAS.
- StaticTemperature cached into `_last_outside_air_temp_k` (`airspeed.cpp:82`, member `airspeed.hpp:83`) but **DEAD** — never written to any topic. Sub still required (shared with baro). No observable effect.
- All three `*_variance` fields ignored.
- `_last_tas_m_s`/`_last_outside_air_temp_k` default 0.0f (`airspeed.hpp:82-83`). IAS before any TAS -> true_airspeed_m_s = 0.0f.
- timestamp uses hrt at decode, NOT CAN frame time (FIXME/HACK `airspeed.cpp:97-103`, TIM5). Observable oracle property.
- NodeInfoPublisher: `registerDeviceCapability(srcNodeId, 0, AIRSPEED)` (`airspeed.cpp:111-114`); device-id arg hardcoded 0.
- Base name `uavcan_airspeed`, ORB_ID(airspeed), NAME=`airspeed` (`airspeed.cpp:43,46`). Subs in ctor (`:47-49`), started in init() error-propagating (`:52-76`).
- float16 decode required for IAS and TAS (IEEE-754 half->float, no extra scale).

---

## 8. Differential pressure

**Source files:**
- `src/drivers/uavcan/sensors/differential_pressure.cpp:45-97`
- `src/drivers/uavcan/sensors/differential_pressure.hpp:43-66`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:54-56`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:124-133`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:149-169`

**DroneCAN message:** `uavcan.equipment.air_data.RawAirData` (DTID 1027).

**uORB topic + direction:** `differential_pressure`, **incoming**.

**Field mapping (differential_pressure):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp_sample | `hrt_absolute_time()` at callback entry | direct (us). `differential_pressure.cpp:70,84` |
| timestamp | `hrt_absolute_time()` at end of callback | direct (us); 2nd call, NOT equal to timestamp_sample, >= it. `differential_pressure.cpp:88` |
| device_id | `make_uavcan_device_id(msg)` | devtype=0x83 (DRV_DIFF_PRESS_DEVTYPE_UAVCAN), address=node_id, bus_type=UAVCAN, bus=iface. `differential_pressure.cpp:85` |
| differential_pressure_pa | `msg.differential_pressure` (f32, Pa) | direct Pa->Pa. If `SENS_DPRES_REV==1`: `-1.0f * msg.differential_pressure` (sign flip). `differential_pressure.cpp:72,78,86` |
| temperature | `msg.static_air_temperature` (f16, K) | Kelvin->degC: `+ kAbsoluteNullCelsius` (deg = K - 273.15). `differential_pressure.cpp:81,87` |
| error_count | not set | 0 (report{} aggregate-init `differential_pressure.cpp:83`) |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_DIFFERENTIAL_PRESSURE` (`Kconfig:53`, default y). Gates `#include` (`sensor_bridge.cpp:54-56`) and instantiation (`sensor_bridge.cpp:125`). `differential_pressure.cpp` always compiled (`CMakeLists.txt:170`).

**Runtime gate:** `UAVCAN_SUB_DPRES` (`uavcan_params.yaml:190`; read `sensor_bridge.cpp:126-127`). bool, **default 0 (DISABLED)**, reboot_required. Code fallback 1 but param default 0. Added if != 0 (`sensor_bridge.cpp:129-131`).

**Redundancy:** Per-source-node via base `publish()` (`sensor_bridge.cpp:256-317`), 4 channels keyed by node_id (`differential_pressure.cpp:90`) -> distinct uORB instance via `orb_advertise_multi`. Exhaustion -> `_out_of_channels`. Each instance's device_id differs by address (node) and bus (iface).

**Device-id encoding:** `make_uavcan_device_id(msg)`: devtype=DRV_DIFF_PRESS_DEVTYPE_UAVCAN=0x83 (ctor `differential_pressure.cpp:52`; `drv_sensor.h:185`); address=node_id; bus_type=UAVCAN; bus=iface. `publish()` separately mutates the bridge's own `_device_id` (`sensor_bridge.cpp:295-296`) but does not alter the populated report.device_id.

**Notes:**
- RawAirData DSDL: f32 static_pressure[Pa], f32 differential_pressure[Pa], f16 static_pressure_sensor_temperature[K], f16 differential_pressure_sensor_temperature[K], f16 static_air_temperature[K], f16 pitot_temperature[K]. This bridge consumes ONLY differential_pressure + static_air_temperature; rest IGNORED.
- **Same RawAirData frame feeds BOTH this bridge AND baro.cpp** (baro maps static_pressure/static_air_temperature to sensor_baro). Port must dispatch RawAirData to both consumers.
- DefaultDataTypeID=1027.
- timestamp and timestamp_sample from two separate hrt calls (lines 70, 88); not equal; timestamp >= timestamp_sample.
- `SENS_DPRES_REV` System bool param, default 0 (`sensor_params.yaml:13-21`), shared with all diff-pressure drivers. Read each callback via param_find/param_get (no cached handle).
- NodeInfoPublisher: registerDeviceCapability(srcNodeID, device_id, DIFFERENTIAL_PRESSURE) (`differential_pressure.cpp:93-96`).
- NAME=`differential_pressure`, base name `uavcan_differential_pressure` (`:45,49`).

---

## 9. Battery

**Source files:**
- `src/drivers/uavcan/sensors/battery.cpp:1-328`
- `src/drivers/uavcan/sensors/battery.hpp:1-129`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:113-122` (factory gate)
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317` (publish/device_id)
- `src/drivers/uavcan/Kconfig:49-51`
- `src/drivers/uavcan/uavcan_params.yaml:169-189`
- `src/lib/battery/battery.cpp:167-199` (getBatteryStatus, Filter path)

**DroneCAN messages:** `uavcan.equipment.power.BatteryInfo` (1092); `ardupilot.equipment.power.BatteryInfoAux` (20004); `cuav.equipment.power.CBAT` (20300).

**uORB topic + direction:** `battery_status` (primary, multi-instance) + `battery_info` (secondary, multi-instance). **Incoming.**

**Field mapping (battery_status unless noted):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `hrt_absolute_time()` (NOT msg) | Raw/RawAux `battery.cpp:121`; CBAT `:231`; Filter via getBatteryStatus() `:185` |
| voltage_v | `BatteryInfo.voltage` [V] (Raw/Aux/Filter); `CBAT.voltage` | direct. Raw `:123`; CBAT `:232`; Filter `updateVoltage` `:309` |
| current_a | `BatteryInfo.current` [A]; `CBAT.current` | Raw direct (`:124`). CBAT NEGATED `-msg.current` (`:233`). Filter `updateCurrent` (`:310`) |
| current_average_a | `CBAT.average_current`; RawAux `getCurrentAverage()`; Filter `_current_average_filter_a` | CBAT NEGATED `-msg.average_current` (`:234`). RawAux (`:201`). Filter (`:172`). Plain Raw -> default/-1 |
| discharged_mah | Raw `sumDischarged(fabsf(BatteryInfo.current))`; CBAT `full_charge_capacity - remaining_capacity` [mAh]; Filter `_discharged_mah` | Raw lib integrates abs(current) `:127`. CBAT subtraction `:235`. Filter `:173` |
| time_remaining_s | Raw NAN; RawAux `computeRemainingTime(fabsf(current))`; CBAT `computeRemainingTime(current_a)`; Filter `computeRemainingTime(current)` | Raw NAN `:128`. RawAux abs `:199-200`. CBAT signed `:261-262`. Filter `:176` |
| remaining | `BatteryInfo.state_of_charge_pct` (Raw/Aux); `CBAT.state_of_charge`; Filter `_state_of_charge` | Raw `/100.0f` `:131`. CBAT `/100.f` `:236`. Filter `:174` |
| scale | const -1.f (Raw & CBAT); Filter `_scale` | Raw `:132`; CBAT `:237`; Filter `:175` |
| temperature | `BatteryInfo.temperature` [K] (Raw & Filter); `CBAT.temperature` [K] | K->C: `+ kAbsoluteNullCelsius`. Raw `:133`; CBAT `:238`; Filter OVERRIDES lib temp with msg-derived `:315` |
| connected | const true (Raw & CBAT); Filter setConnected(true) | Raw `:134`; CBAT `:250`; Filter `:308` -> `:179` |
| source | Raw `BatteryInfo.status_flags & STATUS_FLAG_IN_USE(=1)`; CBAT const SOURCE_EXTERNAL(1); Filter `_source` | Raw masked flag (0/1) `:135`. CBAT `:252`. Filter ctor SOURCE_EXTERNAL (`battery.hpp:123-125`) |
| full_charge_capacity_wh | `BatteryInfo.full_charge_capacity_wh` (Raw); CBAT `full_charge_capacity[mAh]*nominal_voltage/1000` | Raw direct `:136`. CBAT mAh->Wh `:239-240`. Filter forces NAN (`:187`) |
| remaining_capacity_wh | `BatteryInfo.remaining_capacity_wh` (Raw); CBAT `remaining_capacity[mAh]*nominal_voltage/1000` | Raw direct `:137`. CBAT `:241`. Filter forces NAN `:187` |
| id | `BatteryInfo.battery_id` (Raw & Filter); CBAT `msg.getSrcNodeID()` (node id, NOT a CBAT field) | Raw `:138`; Filter `:316`; CBAT node ID `:254` |
| voltage_cell_v[0] | Raw-only (no Aux): `BatteryInfo.voltage` | direct copy of pack voltage into cell[0] (Mavlink2). `:142` |
| cell_count | Raw const 1; RawAux `voltage_cell.size()`; CBAT `CBAT.cell_count`; Filter `_params.n_cells` | Raw hardcoded 1 `:145`. RawAux `min(size,14)` `:184`. CBAT direct `:251`. Filter `:178` |
| warning | `determineWarning(remaining)` (Raw & CBAT); Filter `_warning` | Raw `:148`; CBAT `:268`; uses BAT_LOW/CRIT/EMERGEN_THR (`battery.hpp:96-100`). Filter `:184` |
| cycle_count | `BatteryInfoAux.cycle_count`; `CBAT.cycle_count` | direct. RawAux `:185`; CBAT `:244` |
| over_discharge_count | `BatteryInfoAux.over_discharge_count`; `CBAT.over_discharge_count` | direct. RawAux `:186`; CBAT `:249` |
| nominal_voltage | `BatteryInfoAux.nominal_voltage` (RawAux); `CBAT.nominal_voltage` | RawAux: `(msg.nominal_voltage > FLT_EPSILON) ? msg : NAN` (0 means not provided) `:188`. CBAT direct `:242`. Filter forces NAN |
| is_powering_off | `BatteryInfoAux.is_powering_off`; `CBAT.is_powering_off` | direct. RawAux `:189`; CBAT `:255` |
| capacity | RawAux `full_charge_capacity_wh*1000/nominal_voltage` [mAh] (if nom>EPS); CBAT `CBAT.full_charge_capacity`; Filter `_capacity_mah` | RawAux Wh->mAh guarded by nom>EPS `:191-194` (else unchanged), also `setCapacityMah` `:196`. CBAT direct `:243`. Filter uint16 cast `:182` |
| voltage_cell_v[i] (i<cell_count) | RawAux `BatteryInfoAux.voltage_cell[i]`; CBAT `CBAT.voltage_cell[i]` | direct per-cell. RawAux loop `:203-205`; CBAT loop `:264-266` |
| average_time_to_empty | `CBAT.average_time_to_empty` [min] | direct (CBAT only) `:245` |
| manufacture_date | `CBAT.manufacture_date` | direct (CBAT only) `:246` |
| state_of_health | `CBAT.state_of_health` [%] | direct (CBAT only) `:247` |
| max_error | `CBAT.max_error` [%] | direct (CBAT only) `:248` |
| faults | `CBAT.status_flags` bitmask | remap (CBAT only) `:270-288`: OVERLOAD->FAULT_OVER_CURRENT(3); BAD_BATTERY->FAULT_HARDWARE_FAILURE(9); TEMP_HOT->FAULT_OVER_TEMPERATURE(4); TEMP_COLD->FAULT_UNDER_TEMPERATURE(5). Stored as `(1<<FAULT_*)` OR-accumulated |
| priority | Filter-only `_priority` | getBatteryStatus() `:181` (Raw/CBAT leave 0) |
| internal_resistance_estimate / ocv_estimate / ocv_estimate_filtered / volt_based_soc_estimate / voltage_prediction / prediction_error / estimation_covariance_norm | Filter-only: Battery estimator state | populated only via getBatteryStatus() (`:189-197`); else struct defaults |
| **battery_info**.timestamp | `_battery_status[instance].timestamp` | copied. Raw `:154`; CBAT `:292`; Filter `:321` |
| **battery_info**.id | `_battery_status[instance].id` | direct. Raw `:155`; CBAT `:293`; Filter `:322` |
| **battery_info**.serial_number (char[32]) | Raw/Filter `BatteryInfo.model_instance_id` (uint32, `%PRIu32`); CBAT `CBAT.serial_number` (uint16, `%PRIu16`) | decimal ASCII string. Raw `:153-158` (only if model_instance_id>0); Filter `:320-325` (same guard); CBAT `:294-296` (always) |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_BATTERY` (`Kconfig:49-51`, bool, default y). Guarded `#if defined(...)` include + factory (`sensor_bridge.cpp:51-53,114-122`).

**Runtime gate:** `UAVCAN_SUB_BAT` (`uavcan_params.yaml:169-189`). enum 0=Disable, 1=Raw, 2=Filter; default 0; reboot_required. Factory creates bridge only if != 0 (`sensor_bridge.cpp:114-120`). Inside init() (`battery.cpp:55-66`): if ==FILTER_DATA(2) every instance `_batt_update_mod = Filter`, else Raw. **Same param gates all three subscriptions** (BatteryInfo, BatteryInfoAux, CBAT — all started unconditionally `battery.cpp:68-87` when bridge exists).

**Redundancy:** Per-source-node multi-instance, up to `MAX_INSTANCES = 3` (`BatteryStatus.msg:8`). Two parallel mechanisms:
1. Bridge `_node_ids[3]` (`battery.hpp:113`), 0-init. Each cb linear-scans for slot where `node_id == srcNodeID || == 0` (BatteryInfo `:97-101`, CBAT `:217-221`). BatteryInfoAux matches an EXISTING node_id only (no free-slot claim, `:170-174`). instance>=3 dropped. Slot claimed by `_node_ids[instance]=srcNodeID` (`:113`/`:253`). Per-instance accumulators `_battery_status[3]` and `_battery_info[3]` (`battery.hpp:110-111`) persist ACROSS callbacks — Aux/CBAT augment the struct BatteryInfo filled.
2. Per-instance `_batt_update_mod` (enum {Raw, RawAux, Filter, CBAT}, `battery.hpp:66-71,112`). Precedence: Filter set globally at init, sticky — BatteryInfo->filterData() (`:115-119`), Aux/CBAT early-return if Filter (`:177,224`). CBAT latches: once CBAT for an instance, mode=CBAT (`:229`), subsequent BatteryInfo early-return (`:108-111`); CBAT blocked only by Filter (`:223-224`). Aux upgrades Raw->RawAux (`:182`) but blocked by Filter/CBAT (`:176-180`).

uORB instance alloc done in base `publish()` (`sensor_bridge.cpp:256-317`): `_channels[4]` (`sensor_bridge.hpp:110`) node_id->orb instance; first publish `orb_advertise_multi` (`:299`), thereafter `orb_publish` (`:316`). `battery_info` published on SEPARATE `_battery_info_pub[3]` (`battery.hpp:108`) — independent instance numbering.
node_info: BatteryInfo registers capability keyed by `msg.battery_id` (`:103-106`); CBAT keyed by node_id (`:298-301`), BATTERY.

**Device-id encoding:** NO device_id field in either topic. Base `_device_id` mutated for bookkeeping on first channel create (`sensor_bridge.cpp:295-296`: address=node_id, bus_type=UAVCAN); base ctor sets bus_type=UAVCAN, bus=0; devtype left 0 (set_device_type never called). NOT propagated to topics — instance disambiguation purely via uORB multi-instance index. Richer `make_uavcan_device_id()` NOT used.

**Notes:**
- FOUR decode paths via BatteryDataType. Reproduce mode-selection precedence exactly: Filter global+sticky; CBAT latches+blocks BatteryInfo; Aux upgrades Raw->RawAux blocked by Filter/CBAT.
- `_battery_status[3]`/`_battery_info[3]` PERSISTENT accumulators — keep per-instance state across messages.
- Publish timing: Raw at end of battery_sub_cb (`:151`). RawAux from aux cb ONLY if `_battery_status[instance].timestamp != 0` (BatteryInfo seen) (`:208-210`). CBAT every cb (`:290`). Filter from filterData (`:318`).
- `battery_info` is a SECOND topic (base handles one topic, so `battery.hpp:108` adds own PublicationMulti<battery_info_s>[3]). Published only when serial/model id available: model_instance_id>0 (Raw/Filter) or always (CBAT).
- Temperature K->C via kAbsoluteNullCelsius; Filter OVERWRITES lib temp with msg-derived (`:315`).
- Sign: BatteryInfo.current as-is; discharged_mah uses fabsf. CBAT.current/average_current NEGATED on ingest (`:233-234`), then time_remaining uses signed negative current (`:262`).
- `battery_status.id` inconsistent: Raw/RawAux/Filter use BatteryInfo.battery_id; CBAT uses srcNodeID (`:254`). registerDeviceCapability likewise (battery_id for BatteryInfo `:104-105`, node_id for CBAT `:299-300`).
- FILTER_DATA=2 (`battery.hpp:115`) matches UAVCAN_SUB_BAT enum 2.
- DSDL widths for decoder: BatteryInfo float16 (temperature/voltage/current/remaining_capacity_wh/full_charge_capacity_wh), SOC/health uint7, status_flags uint11, model_name uint8[<32] (unused). BatteryInfoAux.voltage_cell float16[<=255] (length doubles as cell count, clamp 14). CBAT float32; voltage_cell float32[<=15]; SOC/health/max_error uint7.
- Instance caps: logical MAX_INSTANCES=3 via _node_ids; underlying channel table _max_channels=4; battery_info independent counter.
- NO device_id in either output topic — do NOT add one.

---

## 10. Optical flow

**Source files:**
- `src/drivers/uavcan/sensors/flow.cpp:60,62-92`
- `src/drivers/uavcan/sensors/flow.hpp:44-66`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:136-144`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:149-169`
- `src/drivers/uavcan/libdronecan/dsdl/com/hex/equipment/flow/20200.Measurement.uavcan:1-4`

**DroneCAN message:** `com.hex.equipment.flow.Measurement` (DTID 20200, sig 0x6A908866BCB49C18). Vendor (Hex/ProfiCNC). Fields (in order): float32 integration_interval[s]; float32[2] rate_gyro_integral[rad]; float32[2] flow_integral[rad]; uint8 quality.

**uORB topic + direction:** `sensor_optical_flow`, **incoming**.

**Field mapping (sensor_optical_flow):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp_sample | `hrt_absolute_time()` at callback entry (msg has no timestamp; `// TODO`) | direct (local us). `flow.cpp:63` |
| device_id | `make_uavcan_device_id(msg)` | devtype=0x84 (DRV_FLOW_DEVTYPE_UAVCAN), address=node_id, bus_type=UAVCAN(3), bus=iface. `flow.cpp:65` |
| pixel_flow[0] | `msg.flow_integral[0]` (rad) | direct rad->rad, 1:1, no remap/swap |
| pixel_flow[1] | `msg.flow_integral[1]` | direct rad->rad, 1:1 |
| integration_timespan_us | `msg.integration_interval` (s) | `1.e6f * integration_interval` (s->us); float assigned to uint32 (truncation toward zero) |
| quality | `msg.quality` (uint8 0..255) | direct, 1:1 |
| delta_angle[0] | `msg.rate_gyro_integral[0]` IF both [0],[1] PX4_ISFINITE else NAN | direct rad when finite, else NAN |
| delta_angle[1] | `msg.rate_gyro_integral[1]` IF both finite else NAN | direct rad when finite, else NAN |
| delta_angle[2] | constant NAN (msg only 2-axis) | hardcoded NAN in BOTH branches |
| delta_angle_available | true only when both rate_gyro_integral[0]&[1] finite; else struct default false | boolean from finiteness of both |
| max_flow_rate | constant NAN | hardcoded NAN |
| min_ground_distance | constant NAN | hardcoded NAN |
| max_ground_distance | constant NAN | hardcoded NAN |
| timestamp | `hrt_absolute_time()` 2nd call before publish | direct (local us). `flow.cpp:90` |
| distance_m | NOT set | 0 (flow{} zero-init) |
| distance_available | NOT set | false |
| error_count | NOT set | 0 |
| mode | NOT set | 0 (MODE_UNKNOWN) |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_FLOW` (`Kconfig:57`, default y). Guards registration `#if defined(...)` (`sensor_bridge.cpp:136`) and effectively the inclusion of flow.cpp.

**Runtime gate:** `UAVCAN_SUB_FLOW` (`uavcan_params.yaml:199`; bool; **default 0 = DISABLED**; reboot_required). `sensor_bridge.cpp:137-142`: code fallback 1, only instantiated when param != 0.

**Redundancy:** Per-source-node via base `publish()` (`sensor_bridge.cpp:256-317`), 4 channels. Each node_id (`flow.cpp:92`) -> own Channel + own uORB instance via `orb_advertise_multi(ORB_ID(sensor_optical_flow))` (`:299`). Reuse via lookup (`:264-269`) + `orb_publish` (`:316`). After 4 nodes/advertise-fail `_out_of_channels` (`:272-292,304-308`). Channel key is **node_id ONLY**; iface folded into device_id.

**Device-id encoding:** `make_uavcan_device_id(msg)` (`sensor_bridge.hpp:149-169`): devtype=DRV_FLOW_DEVTYPE_UAVCAN=0x84 (`drv_sensor.h:186`), address=node_id, bus_type=UAVCAN(3), bus=iface. set_device_type in ctor (`flow.cpp:44`). The value written to each sample's `flow.device_id` is the per-message make_uavcan_device_id computed at `flow.cpp:65`.

**Notes:**
- DSDL DefaultDataTypeID=20200, signature 0x6A908866BCB49C18. Vendor message, NOT standard uavcan.equipment.
- Only TWO real conversions: (1) integration_interval s->us via *1e6; (2) none for angles. flow_integral rad->pixel_flow rad 1:1; rate_gyro_integral rad->delta_angle rad 1:1; quality 0-255 1:1. NO rad/deg, NO Pa/mbar, NO NED rotation/axis swap/sign flip.
- delta_angle[2] ALWAYS NAN (msg only 2 gyro components). Port MUST hardcode even when 2D gyro valid.
- delta_angle_available gated on BOTH [0] AND [1] finite. When false, bridge explicitly writes delta_angle[0..2]=NAN and leaves delta_angle_available at zero-init false (not explicitly assigned false).
- timestamp and timestamp_sample BOTH from hrt (two calls), NOT msg. timestamp_sample at cb entry (`flow.cpp:63`), timestamp before publish (`:90`).
- integration_timespan_us: uint32 but source computes float (1.e6f * interval) -> implicit float->uint32 truncation (round toward zero). Reproduce.
- Zero-init defaults: distance_m=0, distance_available=false, error_count=0, mode=0.
- NodeInfoPublisher: registerDeviceCapability(node_id, flow.device_id, OPTICAL_FLOW) (`flow.cpp:95-98`).
- Out-of-box DISABLED (UAVCAN_SUB_FLOW default 0) though Kconfig default y.
- Single Subscriber `_sub_flow(node)` started `init()` (`flow.cpp:50`); demuxed by node id in publish().
- NAME=`flow` (`flow.cpp:38`), base name `uavcan_flow` (`:41`).

---

## 11. Rangefinder (UavcanRangefinderBridge)

**Source files:**
- `src/drivers/uavcan/sensors/rangefinder.cpp:43-153`
- `src/drivers/uavcan/sensors/rangefinder.hpp:46-75`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:224-233`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:319-374`
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:103-179`
- `src/lib/drivers/rangefinder/PX4Rangefinder.cpp:40-92`
- `src/lib/drivers/rangefinder/PX4Rangefinder.hpp:41-74`

**DroneCAN message:** `uavcan.equipment.range_sensor.Measurement` (DTID 1050, sig 0x27B69FF7FBCEC600).

**uORB topic + direction:** `distance_sensor`, **incoming**.

**Field mapping (distance_sensor):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `msg.timestamp.usec` else `hrt_absolute_time()` | `timestamp_sample = (usec>0)?usec:hrt` (`rangefinder.cpp:119`) -> `report.timestamp = timestamp_sample` (`PX4Rangefinder.cpp:74`). us direct |
| current_distance | `msg.range` (f16, m) | direct, no scaling. `rangefinder.cpp:120` -> `PX4Rangefinder.cpp:75` |
| signal_quality | `msg.reading_type` (uint3) | quality=-1 default (`:113`); if reading_type==READING_TYPE_VALID_RANGE(1) then 100 (`:115-117`). Post: if quality<0 AND (dist<min OR dist>max) -> 0 (`PX4Rangefinder.cpp:79-83`). Effective: 100 valid; -1 in-bounds non-valid; 0 out-of-bounds non-valid. TOO_CLOSE(2)/TOO_FAR(3)/UNDEFINED(0) -> -1/0 path |
| type | `msg.sensor_type` (uint5) — FIRST msg per bridge only (`_inited`) | switch (`:89-103`): SONAR(1)->ULTRASOUND(1); RADAR(3)->RADAR(3); LIDAR(2)/UNDEFINED(0)/default->LASER(0). Via set_rangefinder_type (`:105`). Set ONCE under _inited (`:85-111`). Default LASER(0) (`PX4Rangefinder.cpp:44`) |
| h_fov | `msg.field_of_view` (f16, rad) — FIRST msg per bridge only | direct rad. set_fov sets BOTH hfov+vfov (`:106` -> `PX4Rangefinder.hpp:54-56`). ONCE under _inited. Default 0 |
| v_fov | `msg.field_of_view` (f16, rad) — FIRST msg per bridge only | direct; same set_fov -> v_fov. beam_orientation_in_body_frame NOT consumed |
| min_distance | param `UAVCAN_RNG_MIN` (NOT msg) | param_get in init() (`:55`), default 0.0 (`yaml:49-56`). Applied once under _inited (`:107`). Also gates signal_quality OOB zeroing |
| max_distance | param `UAVCAN_RNG_MAX` (NOT msg) | param_get in init() (`:56`), default 999.0 (`yaml:57-64`). Applied once under _inited (`:108`). Also gates OOB zeroing |
| orientation | constant ROTATION_DOWNWARD_FACING (NOT msg) | hardcoded (==25) to ctor (`:134` -> `PX4Rangefinder.cpp:43`). Independent of msg.beam_orientation |
| device_id | `make_uavcan_device_id(node_id, iface_index)` | `(0x89<<24)|(node_id<<16)|(iface<<3)|3`. ctor (`:134` -> `PX4Rangefinder.cpp:42`) |
| mode | constant MODE_UNKNOWN (ctor default) | (==0); set_mode never called. `PX4Rangefinder.cpp:45` |
| q[4] | none | NOT written (3-arg update, `:120`); memcpy skipped (`PX4Rangefinder.cpp:86-88`); default (0,0,0,0) |
| variance | none | NOT written anywhere; default 0 |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_RANGEFINDER` (`Kconfig:85-87`, default y). Guards `#include` (`sensor_bridge.cpp:78-80`) and instantiation (`sensor_bridge.cpp:225-233`).

**Runtime gate:** `UAVCAN_SUB_RNG` (`uavcan_params.yaml:270-278`, bool, **DEFAULT 0 = OFF**, reboot_required). `sensor_bridge.cpp:226-231`: local default 1 overwritten by param_get; added if != 0. **Unlike BARO/MAG/GPS (default 1), rangefinder defaults disabled** — no distance_sensor from DroneCAN until UAVCAN_SUB_RNG=1.

**Redundancy:** `get_channel_for_node(msg.getSrcNodeID().get(), msg.getIfaceIndex())` (`rangefinder.cpp:71`), 4 channels. First new node -> `init_driver()` (`:129-152`) constructs NEW PX4Rangefinder per channel with own `make_uavcan_device_id` + ROTATION_DOWNWARD_FACING, `channel->instance = rangefinder->get_instance()`. Each PX4Rangefinder owns `uORB::PublicationMultiData<distance_sensor_s>` (`PX4Rangefinder.hpp:73`) -> separate distance_sensor instance per node. On 4-used/init fail `_out_of_channels` latches (`sensor_bridge.cpp:349-353,360-368`). **Lookup matches node_id ONLY** (`:325`) — two ifaces same node -> first channel. Base publish()/orb_advertise_multi path NOT used.

**Device-id encoding:** `make_uavcan_device_id(node_id, iface_index)` from init_driver (`rangefinder.cpp:132`) -> PX4Rangefinder ctor (`:134` -> `PX4Rangefinder.cpp:42`). bus_type:3=UAVCAN(3); bus:5=iface; address:8=node_id(1..127); devtype:8=DRV_DIST_DEVTYPE_UAVCAN=0x89 (`drv_sensor.h:191`, set_device_type ctor `:49`). Composite = `(0x89<<24)|(node_id<<16)|(iface<<3)|3`. `Measurement.sensor_id` NOT used in device_id.

**Notes:**
- Subscriber type `uavcan::equipment::range_sensor::Measurement` (DTID 1050), started in init() (`rangefinder.cpp:58`).
- **`_inited` latch (`rangefinder.hpp:73`, set `:110`) is PER-BRIDGE** (one bool, NOT per-channel). FIRST Measurement from ANY node configures type/h_fov/v_fov/min/max ONCE for the whole bridge — taken from whichever node's first msg arrives + global UAVCAN_RNG_MIN/MAX, never refreshed. Port must reproduce this single-shot quirk (per-channel sensor_type/fov from later/other nodes ignored).
- Per-sample dynamic fields written EVERY msg: timestamp, current_distance, signal_quality (`:113-120`).
- `msg.sensor_id` and `msg.beam_orientation_in_body_frame` DECODED but UNUSED.
- Publication via PX4Rangefinder's PublicationMultiData (orb_advertise_multi), one instance/channel; base publish() and its _device_id stamping NOT exercised.
- node_info: registerDeviceCapability(SrcNodeID, get_device_id(), RANGEFINDER) (`:122-126`).
- set_device_type(0x89) on bridge base in ctor (`:49`) so get_device_type() returns 0x89 in make_uavcan_device_id. Full device_id (with devtype) passed straight into PX4Rangefinder ctor.
- 5th distinct node latches _out_of_channels and is dropped.

---

## 12. Hygrometer (UavcanHygrometerBridge)

**Source files:**
- `src/drivers/uavcan/sensors/hygrometer.cpp:39-80`
- `src/drivers/uavcan/sensors/hygrometer.hpp:41-62`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:69-70` (include)
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:179-186` (registration)
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317` (publish/channel alloc)
- `src/drivers/uavcan/sensors/sensor_bridge.hpp:88-179`
- `src/drivers/uavcan/Kconfig:73-75`
- `src/drivers/uavcan/uavcan_params.yaml:233-241`
- `src/drivers/uavcan/CMakeLists.txt:183`
- `src/drivers/uavcan/libdronecan/dsdl/dronecan/sensors/hygrometer/1032.Hygrometer.uavcan`
- `src/drivers/drv_sensor.h:192` (DRV_HYGRO_DEVTYPE_UAVCAN=0x8A)
- `src/lib/atmosphere/atmosphere.h:54`

**DroneCAN message:** `dronecan.sensors.hygrometer.Hygrometer` (DTID 1032). Fields: float16 temperature (DSDL comment "degrees C"); float16 humidity ("percentage"); uint8 id ("0 for first sensor").

**uORB topic + direction:** `sensor_hygrometer`, **incoming**.

**Field mapping (sensor_hygrometer):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp_sample | `hrt_absolute_time()` at callback entry | direct (us). `hygrometer.cpp:63,67` |
| device_id | `make_uavcan_device_id(msg)` | devtype=0x8A (DRV_HYGRO_DEVTYPE_UAVCAN), address=node_id, bus_type=UAVCAN, bus=iface. `hygrometer.cpp:68` |
| temperature | `msg.temperature` (f16) | `msg.temperature + kAbsoluteNullCelsius` (i.e. treated as Kelvin -> Celsius, -273.15). **Contradicts DSDL "degrees C" comment** — port MUST replicate the +(-273.15f) offset. Result Celsius. `hygrometer.cpp:69` |
| humidity | `msg.humidity` (f16) | direct copy (percentage). `hygrometer.cpp:70` |
| timestamp | `hrt_absolute_time()` 2nd call at publish | direct (us); NOT equal to timestamp_sample. `hygrometer.cpp:71` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_HYGROMETER` (`Kconfig:73-75`, default y). Guards `#include` (`sensor_bridge.cpp:69-70`) and registration (`sensor_bridge.cpp:180-186`). `hygrometer.cpp` always compiled (`CMakeLists.txt:183`); only subscriber instantiation gated.

**Runtime gate:** `UAVCAN_SUB_HYGRO` (bool, default 0, reboot_required; `uavcan_params.yaml:233-241`). `sensor_bridge.cpp:181-185`: code default 1 if param missing; added only if != 0. By default param value bridge is OFF.

**Redundancy:** Per-source-node via base. Each node -> own Channel (`sensor_bridge.hpp:90-96`), 4 channels (default ctor `hygrometer.cpp:42`). First msg -> `publish()` (`sensor_bridge.cpp:257-317`) finds by node_id else allocates + `orb_advertise_multi` -> new instance. Exhaustion -> `_out_of_channels`. Keyed on `msg.getSrcNodeID().get()` (`hygrometer.cpp:73`). Capability HYGROMETER registered per src node (`:76-79`) when non-null. set_device_type in ctor (`:45`). Base `_device_id.address` mutated on new channel (`sensor_bridge.cpp:294-296`) but per-report device_id is the make_uavcan_device_id value (authoritative).

**Device-id encoding:** `make_uavcan_device_id(msg)` (`hygrometer.cpp:68`): devtype=DRV_HYGRO_DEVTYPE_UAVCAN=0x8A (`drv_sensor.h:192`); address=node_id; bus_type=UAVCAN; bus=iface. Ctor also set_device_bus_type(UAVCAN)/set_device_bus(0) on base but published id is the make_uavcan_device_id value.

**Notes:**
- Subscriber `uavcan::Subscriber<dronecan::sensors::hygrometer::Hygrometer>` `_sub_hygro` (`hygrometer.hpp:60`), started init() (`hygrometer.cpp:50`).
- NAME=`hygrometer_sensor` (`:39`); base name `uavcan_hygrometer_sensor` (`:42`).
- **CRITICAL unit conversion:** temperature = `msg.temperature - 273.15f` (K->C). Do NOT pass through. DSDL says degrees C but PX4 treats as Kelvin. Use -273.15f.
- humidity straight passthrough (percentage), no scaling.
- DroneCAN `id` (uint8) NOT mapped; ignored (PX4 distinguishes by node id + iface).
- Two hrt calls: timestamp_sample (cb entry `:63/67`) and timestamp (publish `:71`).
- float16 on wire; decode f16->f32 for temperature/humidity before offset.
- report{} zero-init; uORB has only the 5 listed fields.
- No init_driver override / no class device; uses simple publish() path, not get_channel_for_node().

---

## 13. Fuel tank (UavcanFuelTankStatusBridge)

**Source files:**
- `src/drivers/uavcan/sensors/fuel_tank_status.cpp:43,45-48,50-67,69-86,88-91`
- `src/drivers/uavcan/sensors/fuel_tank_status.hpp:47-73`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:60-61`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:146-155`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256-317`

**DroneCAN message:** `uavcan.equipment.ice.FuelTankStatus` (DTID 1129).

**uORB topic + direction:** `fuel_tank_status`, **incoming**.

**Field mapping (fuel_tank_status):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `hrt_absolute_time()` | receive time (us), not msg. `fuel_tank_status.cpp:73` |
| maximum_fuel_capacity | param `UAVCAN_ECU_MAXF` (float, liters, default 15.0) | `_max_fuel_capacity * 1000.0f` -> ml. NOT from msg. `:74,61` |
| fuel_type | param `UAVCAN_ECU_FUELT` (enum int32, default 1=Liquid) | `static_cast<uint8_t>(_fuel_type)`; MAV_FUEL_TYPE (0=UNKNOWN,1=LIQUID,2=GAS). NOT from msg. `:75,64` |
| consumed_fuel | constant NAN | hardcoded — only remaining measured. `:76` |
| fuel_consumption_rate | `msg.fuel_consumption_rate_cm3pm` (f32, cm^3/min) | `/ 60.0f` -> ml/s (cm^3==ml, per-min->per-sec). `:77` |
| percent_remaining | `msg.available_fuel_volume_percent` (uint7, 0..100) | direct (uint7->uint8). `:78` |
| remaining_fuel | `msg.available_fuel_volume_cm3` (f32, cm^3) | direct (cm^3==ml, no scale). `:79` |
| fuel_tank_id | `msg.fuel_tank_id` (uint8) | direct. `:80` |
| temperature | `msg.fuel_temperature` (f16, K, optional) | `!PX4_ISFINITE ? NAN : msg.fuel_temperature` — pass-through Kelvin, NaN if non-finite. `:83` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_FUEL_TANK_STATUS` (`Kconfig:61-63`, default y). Gates `#include` + registration (`sensor_bridge.cpp:60-61`, `147-155`) and source compile (`CMakeLists.txt:175`).

**Runtime gate:** `UAVCAN_SUB_FUEL` (bool, default 0 = disabled; `uavcan_params.yaml:206-212`, reboot_required). `sensor_bridge.cpp:148-153`: local int32 inited 1, overwritten by param_get; constructed only if != 0. Param default 0 -> OFF unless UAVCAN_SUB_FUEL=1.

**Redundancy:** Per-source-node via base `publish(node_id, report)` (`sensor_bridge.cpp:256-317`). Channel per srcNodeID (`fuel_tank_status.cpp:85`): first msg -> free channel + `orb_advertise_multi` -> new instance; reuse + orb_publish thereafter. `_max_channels` caps concurrent nodes; exhaustion -> `_out_of_channels`. No fusion/voting. Does NOT use CDev/get_channel_for_node path: constructed with device-id-name = nullptr (`UavcanSensorBridgeBase("uavcan_fuel_tank_status", ORB_ID(fuel_tank_status), nullptr)`, `:46`) and init_driver() is a no-op returning PX4_OK (`:88-91`).

**Device-id encoding:** Base `_device_id` updated in publish() on new channel (`sensor_bridge.cpp:294-296`): address=node_id, bus_type=UAVCAN. devtype and other fields left 0. **fuel_tank_status has NO device_id field** — computed id not copied into report; published payload carries no device id.

**Notes:**
- DefaultDataTypeID=1129 (`FuelTankStatus.hpp:171`). DSDL `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/ice/1129.FuelTankStatus.uavcan`.
- DSDL fields: void9 (reserved, skip); uint7 available_fuel_volume_percent; f32 available_fuel_volume_cm3; f32 fuel_consumption_rate_cm3pm (may be negative during transfer/refuel); f16 fuel_temperature (K, optional NaN); uint8 fuel_tank_id. **Reimplementation must skip leading void9 padding.**
- Unit semantics: DroneCAN cm^3 + cm^3/min; uORB ml + ml/s. 1 cm^3 = 1 ml so volume direct; rate /60. maximum_fuel_capacity from liters param x1000, NOT msg.
- maximum_fuel_capacity and fuel_type sourced ENTIRELY from params (UAVCAN_ECU_MAXF, UAVCAN_ECU_FUELT), fetched once in init() (`:60-65`), NOT decoded from msg. Port must inject into every report.
- consumed_fuel always NAN.
- temperature is the only field with presence check (PX4_ISFINITE else NAN). percent_remaining/remaining_fuel/fuel_tank_id/fuel_consumption_rate copied unconditionally.
- temperature is float32 Kelvin in uORB, widening copy from f16 K, no offset/scale (NOT degC).
- Generated storage types (`FuelTankStatus.hpp:95-99`): percent uint7, volume_cm3 f32, rate f32, fuel_temperature f16, fuel_tank_id uint8. Tail-array-optimization on fuel_tank_id (`:285`).
- Standard broadcast subscriber (`fuel_tank_status.hpp:69`); no service/anonymous frames.

---

## 14. ICE status

**Source files:**
- `src/drivers/uavcan/sensors/ice_status.cpp:43,60,91`
- `src/drivers/uavcan/sensors/ice_status.hpp:44`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:190`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:256`

**DroneCAN messages:** `uavcan.equipment.ice.reciprocating.Status` (DTID 1120); nested `CylinderStatus[<=16] cylinder_status`.

**uORB topic + direction:** `internal_combustion_engine_status`, **incoming**.

**Field mapping (internal_combustion_engine_status):**

| uORB field | source field/expr | transform |
|---|---|---|
| timestamp | `hrt_absolute_time()` (NOT msg.timestamp) | direct (us). `ice_status.cpp:64` |
| state | `msg.state` | direct (uint2->uint8; STOPPED=0/STARTING=1/RUNNING=2/FAULT=3). `:65` |
| flags | `msg.flags` | direct (uint30->uint32; FLAG_* identical). `:66` |
| engine_load_percent | `msg.engine_load_percent` | direct (uint7->uint8, %[0,127]). `:67` |
| engine_speed_rpm | `msg.engine_speed_rpm` | direct (uint17->uint32, rpm). `:68` |
| spark_dwell_time_ms | `msg.spark_dwell_time_ms` | direct (f16->f32, ms). `:69` |
| atmospheric_pressure_kpa | `msg.atmospheric_pressure_kpa` | direct (f16->f32, kPa; NO Pa/mbar/kPa conv). `:70` |
| intake_manifold_pressure_kpa | `msg.intake_manifold_pressure_kpa` | direct (f16->f32, kPa). `:71` |
| intake_manifold_temperature | `msg.intake_manifold_temperature` | direct (f16->f32, kelvin; no K/C conv). `:72` |
| coolant_temperature | `msg.coolant_temperature` | direct (f16->f32, kelvin). `:73` |
| oil_pressure | `msg.oil_pressure` | direct (f16->f32, kPa). `:74` |
| oil_temperature | `msg.oil_temperature` | direct (f16->f32, kelvin). `:75` |
| fuel_pressure | `msg.fuel_pressure` | direct (f16->f32, kPa). `:76` |
| fuel_consumption_rate_cm3pm | `msg.fuel_consumption_rate_cm3pm` | direct (f32, cm^3/min). `:77` |
| estimated_consumed_fuel_volume_cm3 | `msg.estimated_consumed_fuel_volume_cm3` | direct (f32, cm^3). `:78` |
| throttle_position_percent | `msg.throttle_position_percent` | direct (uint7->uint8, %). `:79` |
| ecu_index | `msg.ecu_index` | direct (uint6->uint8). `:80` |
| spark_plug_usage | `msg.spark_plug_usage` | direct (uint3->uint8; SINGLE=0/FIRST=1/SECOND=2/BOTH=3). `:81` |
| ignition_timing_deg | `msg.cylinder_status[0].ignition_timing_deg` | direct (f16->f32, crankshaft deg); GATED on `cylinder_status.size() > 0` else default 0 (value-init `:63`); ONLY cylinder 0. `:83-84` |
| injection_time_ms | `msg.cylinder_status[0].injection_time_ms` | direct (f16->f32, ms); GATED size>0 else 0. `:85` |
| cylinder_head_temperature | `msg.cylinder_status[0].cylinder_head_temperature` | direct (f16->f32, kelvin); GATED size>0 else 0. `:86` |
| exhaust_gas_temperature | `msg.cylinder_status[0].exhaust_gas_temperature` | direct (f16->f32, kelvin); GATED size>0 else 0. `:87` |
| lambda_coefficient | `msg.cylinder_status[0].lambda_coefficient` | direct (f16->f32, dimensionless); GATED size>0 else 0. `:88` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_ICE_STATUS` (`Kconfig:77-79`, default y). Inside `if DRIVERS_UAVCAN` (`Kconfig:8`) which `depends on PLATFORM_NUTTX` (`:4`). Guards `#include` (`sensor_bridge.cpp:72-74`) and instantiation (`sensor_bridge.cpp:191-199`).

**Runtime gate:** `UAVCAN_SUB_ICE` (bool). `sensor_bridge.cpp:192-197`: code default 1, registered param default 0 (`uavcan_params.yaml:242-250`, reboot_required) -> OFF unless enabled. Bridge constructed with node only (no NodeInfoPublisher; nullptr to base, `ice_status.cpp:44`).

**Redundancy:** Per-source-node via base. publish() (`sensor_bridge.cpp:256-317`) keyed on node_id (`ice_status.cpp:91`). `_max_channels=4` (default ctor). First new node -> Channel + `orb_advertise_multi` -> new instance (`:299`); subsequent orb_publish to matched channel (linear lookup `:264-269`). After 4 nodes/advertise-fail `_out_of_channels` (`:273-275,288-292,304-308`). No iface merging — each node_id independent instance.

**Device-id encoding:** Topic has NO device_id field — NOT serialized, NOT a verifiable output. Base publish() mutates inherited `_device_id` on channel create (`sensor_bridge.cpp:295-296`: address=node_id, bus_type=UAVCAN); base ctor sets bus_type=UAVCAN, bus=0; devtype never set (UavcanIceStatusBridge never calls set_device_type; init_driver no-op `ice_status.cpp:94-97`). make_uavcan_device_id NOT used.

**Notes:**
- INCOMING ONLY. No outgoing/PUB path.
- timestamp = hrt at receive (`:64`); msg has no timestamp field.
- report value-initialized via `::internal_combustion_engine_status_s()` (`:63`) — unassigned fields 0. Relevant for the 5 cylinder fields when cylinder_status empty (stay 0, NOT NaN, even though DSDL says unknown=NaN).
- Cylinder array up to 16 (CylinderStatus[<=16]); bridge consumes ONLY [0]. 1..15 ignored.
- NO unit conversions anywhere: pressures kPa, temperatures kelvin, angle degrees, rpm/percent/ms direct. No NED/rad-deg/Pa-mbar. Port must NOT add scaling.
- Narrow widths (uint2/7/17/6/3/30) widened to uint8/uint32 by plain assignment.
- All f16 message fields decoded to f32 by libuavcan; port must do same f16->f32 expansion.
- init() starts subscriber with IceStatusCbBinder -> ice_status_sub_cb (`:48-58`); 0 on success.
- Base device name `uavcan_ice_status` (`:44`); NAME=`ice_status` (`:41`).
- DSDL: `.../uavcan/equipment/ice/reciprocating/1120.Status.uavcan` and `CylinderStatus.uavcan`.

---

## 15. Safety button

**Source files:**
- `src/drivers/uavcan/sensors/safety_button.cpp:43,60,94`
- `src/drivers/uavcan/sensors/safety_button.hpp:41`
- `src/drivers/uavcan/sensors/sensor_bridge.cpp:235`
- `src/lib/button/ButtonPublisher.cpp:50`
- `src/lib/button/ButtonPublisher.hpp:73`

**DroneCAN message:** `ardupilot.indication.Button` (DSDL id 20001, `dsdl/ardupilot/indication/20001.Button.uavcan`). Fields: uint8 button [BUTTON_SAFETY=1], uint8 press_time [0.1s units, saturates 255]; sent at 10Hz while pressed.

**uORB topic + direction:** `button_event` (published via `ORB_ID(safety_button)`, an alias/second instance declared by `# TOPICS button_event safety_button` in `msg/ButtonEvent.msg:4`). **Incoming.**

**Field mapping (button_event):**

| uORB field | source field/expr | transform |
|---|---|---|
| triggered | constant `true` — set when `msg.button == BUTTON_SAFETY && msg.press_time >= 10`; event flag, not a msg value | constant true (ButtonPublisher::safetyButtonTriggerEvent, `ButtonPublisher.cpp:55`) |
| timestamp | `hrt_absolute_time()` at publish (Button DSDL has no timestamp) | us. `ButtonPublisher.cpp:56` |

**Compile gate:** `CONFIG_UAVCAN_SENSOR_SAFETY_BUTTON` (`Kconfig:89-91`, default y). Guards registration `#if defined(...)` (`sensor_bridge.cpp:237`). The .cpp unconditionally listed in `CMakeLists.txt:184`; only registration gated.

**Runtime gate:** `UAVCAN_SUB_BTN` (int32, default 0 / disabled, reboot_required; `uavcan_params.yaml:279-287`). Read `sensor_bridge.cpp:238-239`; bridge added if != 0 (`:241-242`). Per-bridge YAML default 0 even though Kconfig default y.

**Redundancy:** None. Bypasses UavcanSensorBridgeBase channel allocation: init_driver() no-op returning PX4_OK (`safety_button.cpp:94-97`); callback never calls publish()/get_channel_for_node(). Publishes directly through a single ButtonPublisher (one `uORB::Publication` to `ORB_ID(safety_button)`). All Button messages from any node ID collapse into the same single publication — no multi-instance, no per-node instancing, no node_id filtering. Pairing-counter state (_pairing_button_counter, _start_timestamp, _new_press_timestamp) global to the bridge.

**Device-id encoding:** Not encoded. Base ctor called with device-id/name = nullptr (`UavcanSensorBridgeBase("uavcan_safety_button", ORB_ID(button_event), nullptr)`, `safety_button.cpp:44`); publish() never invoked so base device_id population never runs. `button_event_s` has no device_id field. ButtonPublisher uses a plain single-instance `uORB::Publication`.

**Notes:**
- Trigger logic (`safety_button.cpp:63-69`): `is_safety = (msg.button == BUTTON_SAFETY == 1)`; `pressed = (msg.press_time >= 10)` i.e. >= 1.0s (press_time in 0.1s). Only when is_safety && pressed is safetyButtonTriggerEvent() called — the sole path that publishes triggered=true. Port must reproduce both `==BUTTON_SAFETY` and `>=10` thresholds.
- Secondary PAIRING path (`:72-91`) also gated on is_safety but does NOT publish button_event. Counts discrete presses: 2s idle resets counter (_start_timestamp); each new press (elapsed > msg.press_time*100*1000 us) increments counter; on reaching PAIRING_BUTTON_EVENT_COUNT(=3, `ButtonPublisher.hpp:70`) pairingButtonTriggerEvent() fires, publishing vehicle_command (VEHICLE_CMD_START_RX_PAIR, param1=10.0), led_control (mask 0xff, MODE_BLINK_FAST, COLOR_WHITE, 20 blinks, priority 2), tune_control (TUNE_ID_NOTIFY_POSITIVE) (`ButtonPublisher.cpp:61-84`). Out of scope for the button_event oracle but must be preserved.
- press_time -> us as `msg.press_time * 100 * 1000` (`:79`) for pairing inter-press timing only; NOT used for any button_event field.
- `button_event_s` fields are ONLY `{uint64 timestamp; bool triggered}` (`msg/ButtonEvent.msg:1-2`). ORB_QUEUE_LENGTH=2.
- Subscriber `uavcan::Subscriber<ardupilot::indication::Button, ButtonCbBinder> _sub_button` started init() (`:48-58`).
- ButtonPublisher ctor calls `_safety_button_pub.advertise()` (`ButtonPublisher.cpp:47`) — advertised at construction.
- No NED/rad-deg/Pa-mbar conversions; only press_time 0.1s->us scaling for pairing timing.

---

## 16. ESC

**Source files:**
- `src/drivers/uavcan/actuators/esc.cpp:48-234`
- `src/drivers/uavcan/actuators/esc.hpp:60-138`
- `src/drivers/uavcan/uavcan_main.cpp:531-543` (runtime gate + init)
- `src/drivers/uavcan/uavcan_main.cpp:1092-1135` (UavcanMixingInterfaceESC: updateOutputs/Run/mixerChanged)
- `src/drivers/uavcan/uavcan_main.hpp:123-147,268-270` (class + MixingOutput member, compile gate)
- `src/drivers/uavcan/module.yaml:84-91` (UAVCAN_EC range params), `:8-29` (UAVCAN_ESC_IFACE)
- `src/drivers/uavcan/uavcan_params.yaml:5-22` (UAVCAN_ENABLE)
- `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/esc/1030.RawCommand.uavcan:1-7`, `1034.Status.uavcan:1-20`, `1036.StatusExtended.uavcan:1-16`
- `msg/EscStatus.msg:1-33`, `msg/EscReport.msg:1-31`

**DroneCAN messages:**
- OUT `uavcan.equipment.esc.RawCommand` (1030): int14[<=20] cmd; range [-8192,8191], MaxSize=20
- IN `uavcan.equipment.esc.Status` (1034): uint32 error_count; f16 voltage[V]; f16 current[A]; f16 temperature[K]; int18 rpm; uint7 power_rating_pct; uint5 esc_index
- IN `uavcan.equipment.esc.StatusExtended` (1036): uint7 input_pct; uint7 output_pct; int9 motor_temperature_degC; uint9 motor_angle; uint19 status_flags; uint5 esc_index

**uORB topic + direction:** `esc_status` (publishes EscStatus embedding EscReport[12] esc; subscribes actuator_motors/actuator_outputs indirectly via MixingOutput for outgoing RawCommand; also subscribes dronecan_node_status + device_information for failure derivation). **Bidirectional.**

**Field mapping:**

| uORB field | source field/expr | transform |
|---|---|---|
| OUT: RawCommand.cmd[i] | `outputs[i]` (normalized motor cmd from MixingOutput, group UAVCAN_EC, scaled to [UAVCAN_EC_MINx..MAXx], default [1..8191]) | `static_cast<int>(lroundf(outputs[i]))`; push_back for i in [0, output_array_size). output_array_size = highest mapped index+1 (`uavcan_main.cpp:1101-1106`). Clamped int14 [-8192..8191]. `esc.cpp:101-103` |
| OUT: RawCommand priority | constant | TransferPriority::NumericallyMin (highest); ctor `esc.cpp:54`. Iface mask from UAVCAN_ESC_IFACE applied `esc.cpp:77-81` |
| OUT: publish rate | MAX_RATE_HZ=400 | skip if `(now - _prev_cmd_pub) < 1e6/400 = 2500us`. `esc.cpp:91-97`. Also setMaxTopicUpdateRate(1e6/400) `uavcan_main.cpp:123` |
| esc[esc_index].timestamp | `hrt_absolute_time()` | direct (local clock at Status reception). Guarded esc_index<CONNECTED_ESC_MAX(12). `esc.cpp:115-117` |
| esc[esc_index].esc_voltage | `Status.voltage` | direct (V, f16->f32). `esc.cpp:118` |
| esc[esc_index].esc_current | `Status.current` | direct (A; may be negative for regen). `esc.cpp:119` |
| esc[esc_index].esc_temperature | `Status.temperature` | K->C: `+ kAbsoluteNullCelsius`. `esc.cpp:120` |
| esc[esc_index].esc_rpm | `Status.rpm` | direct (int18->int32; negative=reverse). `esc.cpp:122` |
| esc[esc_index].esc_errorcount | `Status.error_count` | direct (uint32). `esc.cpp:123` |
| esc[esc_index].failures | derived from dronecan_node_status (health/vendor_specific_status_code for srcNodeID) + device_information | `get_failures(esc_index, srcNodeID)`: 0 if HEALTH_OK/WARNING. If ERROR/CRITICAL: if device_information matches (device_type==ESC, device_id==esc_index, name contains 'iq_motion') decode vendor bits via bit_to_failure_map (bits0-2,11->OVER_VOLTAGE; 3,4->OVER_CURRENT; 5->OVER_ESC_TEMP; 6->MOTOR_OVER_TEMP; 7->GENERIC; 8->OVER_RPM; 9->WARN_ESC_TEMP; 10->MOTOR_WARN_TEMP) as (1<<failure_type); else (1<<FAILURE_GENERIC). `esc.cpp:124,171-234` |
| esc[index].motor_temperature | `StatusExtended.motor_temperature_degC` | direct (Celsius, int9->int16). `esc.cpp:150` |
| esc[index].esc_power | `StatusExtended.input_pct` | direct (% 0..100, uint7->int8). `esc.cpp:151` |
| esc[i].actuator_function | `_mixing_output.outputFunction(i)` | direct cast (uint8_t); per-channel in mixerChanged() for i<CONNECTED_ESC_MAX. `uavcan_main.cpp:1129-1131` |
| esc[*].esc_state | (none) | NOT written; default 0 |
| esc_count | `_rotor_count` | direct. _rotor_count = count of set output functions, set via set_rotor_count() from mixerChanged(). `esc.cpp:126,108-111`; `uavcan_main.cpp:1124-1134` |
| counter | internal | `_esc_status.counter += 1` per Status publish. `esc.cpp:127` |
| esc_connectiontype | constant | ESC_CONNECTION_TYPE_CAN (=4). `esc.cpp:128` |
| esc_online_flags | per-channel freshness | check_escs_status(): bit i set if `esc[i].timestamp>0 && (now - esc[i].timestamp) < 1200ms`; over CONNECTED_ESC_MAX. `esc.cpp:129,155-169` |
| esc_armed_flags | `_rotor_count` | `(1 << _rotor_count) - 1`. `esc.cpp:130` |
| timestamp | esc_report.timestamp | = hrt at Status reception (same as esc[esc_index].timestamp). `esc.cpp:131` |

**Compile gate:** `CONFIG_UAVCAN_OUTPUTS_CONTROLLER` (`Kconfig:17-19`, default y, "Include servo & ESC controller"). Guards UavcanEscController/_esc_controller + UavcanMixingInterfaceESC (`uavcan_main.hpp:123,176,268-270`) and init/updateOutputs (`uavcan_main.cpp:532-545,1092`). **This single gate also enables the Servo controller.**

**Runtime gate:** `UAVCAN_ENABLE > 2` (i.e. ==3, "Sensors and Actuators (ESCs) Automatic Config"). Checked `uavcan_main.cpp:533-538` before `_esc_controller.init()`. **NO dedicated UAVCAN_SUB_ESC/UAVCAN_PUB_ESC param** — Status/StatusExtended subscribers started UNCONDITIONALLY in init() (`esc.cpp:60,68`), reception gated only by UAVCAN_ENABLE>2. Output channels via per-channel function params. Outgoing CAN interface via UAVCAN_ESC_IFACE bitmask (default 255, `module.yaml:8-29`).

**Redundancy:** Per-ESC channel allocation by **esc_index field IN the Status/StatusExtended message** (uint5, 0=first), NOT by source node ID — esc_index directly indexes `esc_status_s::esc[esc_index]` (`esc.cpp:115-116,145-148`). Source node ID used only for failure derivation (matching dronecan_node_status.node_id) and node_info registration. Single shared esc_status_s aggregates up to CONNECTED_ESC_MAX=12 ESCs; RawCommand.cmd[] indexed by motor/output index. Single esc_status uORB instance via PublicationMulti. Online/offline per-channel via 1200ms timeout.

**Device-id encoding:** No device_id on the published esc_status topic: published via `uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)}` (`esc.hpp:122`) advertising multi-instance (`esc.cpp:75`) but esc_status_s has no device_id field. Only 'device_id' usage is INPUT side: (1) get_failures matches `device_information_s.device_id == esc_index` (`esc.cpp:201`); (2) node-info registration uses device_id = `msg.esc_index` with source node_id = srcNodeID to call registerDeviceCapability(node_id, esc_index, ESC) (`esc.cpp:136-140`).

**Notes:**
- Outgoing scaling chain: control_allocator/actuator_motors -> MixingOutput (prefix 'UAVCAN_EC', 12 channels, member `uavcan_main.hpp:146`) maps each normalized output to integer [UAVCAN_EC_MINx..MAXx] (`module.yaml:84-91`; per-channel default min=1, max=8191, range cap 0..8191) -> update_outputs() rounds via lroundf, pushes int14 cmd[]. max_output_value()=8191 (int14, `esc.hpp:85`).
- output_array_size optimization: only first (highest_mapped_index+1) cmd entries sent (`uavcan_main.cpp:1099-1108`).
- ESC temperature K->C via kAbsoluteNullCelsius (`esc.cpp:120`). motor_temperature already Celsius (no conv).
- static_assert RawCommand cmd MaxSize(20) >= MAX_ACTUATORS(=CONNECTED_ESC_MAX=12) (`esc.hpp:66`).
- esc_index bounds check uses CONNECTED_ESC_MAX(12); cmd[] supports 20; MAX_ACTUATORS=12.
- esc_state never populated (stays 0).
- Failure decode 'iq_motion'/VertiQ special-case maps bit 11 to OVER_VOLTAGE (duplicate of bits 0-2); FAILURE_INCONSISTENT_CMD(4)/MOTOR_STUCK(5) not produced.
- dronecan_node_status and device_information are uORB inputs (not direct DroneCAN messages decoded here); node health from NodeStatus bridge elsewhere.
- Fixed 400Hz (MAX_RATE_HZ) for both publish and MixingOutput max topic rate ('TODO: configurable').
- Same CONFIG_UAVCAN_OUTPUTS_CONTROLLER gate and UAVCAN_ENABLE>2 also enable Servo; keep ESC and Servo coupled or document divergence.

---

## 17. Servo

**Source files:**
- `src/drivers/uavcan/actuators/servo.cpp:47,55,60`
- `src/drivers/uavcan/actuators/servo.hpp:48,49,50,59`
- `src/drivers/uavcan/uavcan_main.hpp:156,174,272,273`
- `src/drivers/uavcan/uavcan_main.cpp:89,91,124,418,455,1137,1143`
- `src/drivers/uavcan/module.yaml:92`
- `src/drivers/uavcan/Kconfig:17`
- `src/lib/mixer_module/functions/FunctionServos.hpp:50,60,64`
- `src/lib/mixer_module/mixer_module.cpp:527,535,560`
- `src/drivers/uavcan/libdronecan/dsdl/uavcan/equipment/actuator/Command.uavcan:6`, `1010.ArrayCommand.uavcan:6`

**DroneCAN messages:** `uavcan.equipment.actuator.ArrayCommand` (DTID 1010); `uavcan.equipment.actuator.Command` (nested).

**uORB topic + direction:** `actuator_servos` (subscribed input). **Outgoing.**

**Field mapping:**

| uORB field / DroneCAN out | source field/expr | transform |
|---|---|---|
| ArrayCommand.commands[i].command_value (OUT, f16) | `actuator_servos.control[i]` (i=0..7, [-1,1], NaN=disarmed) | 2-stage. STAGE 1 (MixingOutput, prefix UAVCAN_SV, 8ch): control[i] [-1,1] -> outputs[i] via `interpolate(value,-1,1, MIN, MAX)` (`mixer_module.cpp:560`), defaults MIN=UAVCAN_SV_MINi(0), MAX=UAVCAN_SV_MAXi(1000) (`module.yaml:97-98`): -1->0, 0->500, +1->1000. NaN -> DIS=UAVCAN_SV_DISi(500) (`module.yaml:96`; `mixer_module.cpp:539`). Reverse via UAVCAN_SV_REVi negates before interpolation (`:542-544`). Optional 3-point interpolateNXY({-1,0,1},{MIN,CENTER,MAX}) only if center set and 800<=center<=2200 (`:548-556`). Lockdown/kill->DIS; termination->failsafe (`:491-501`). STAGE 2 (`servo.cpp:55`): `command_value = outputs[i]/500.f - 1.f`. Default end-to-end: [-1,1]->[-1,1] (UNITLESS); disarmed(500)->0.0. float16 on wire |
| ArrayCommand.commands[i].actuator_id | channel index i (0-based loop, num_outputs <= 8) | `cmd.actuator_id = i` (`servo.cpp:53`); 0-based output index |
| ArrayCommand.commands[i].command_type | (constant) | fixed COMMAND_TYPE_UNITLESS(=0) (`servo.cpp:54`; `Command.uavcan:11`) |
| ArrayCommand.commands (length) | one Command per output channel | i in [0,num_outputs); num_outputs = _max_num_outputs from MixingOutput (`mixer_module.cpp:527`), capped MAX_ACTUATORS=8 (`servo.hpp:48`). DSDL bounds <=15, so no truncation. `servo.cpp:51-58` |

**Compile gate:** `CONFIG_UAVCAN_OUTPUTS_CONTROLLER` (`Kconfig:17`, bool "Include servo & ESC controller", default y). Guards instantiation of _servo_controller & _mixing_interface_servo (`uavcan_main.cpp:89-92`), setMaxTopicUpdateRate (`:124`), updateParams (`:416-419`), ScheduleNow (`:453-456`), updateOutputs/Run (`~1137-1150`). Shared with ESC.

**Runtime gate:** NONE dedicated. No UAVCAN_SUB_*/UAVCAN_PUB_* param for servo (confirmed: no param_find for a servo sub/pub gate under `src/drivers/uavcan/actuators/`). Activated implicitly per-channel by assigning UAVCAN_SV_FUNCi to a Servo function (Servo1..ServoMax) so MixingOutput allocates FunctionServos and subscribes to actuator_servos (`mixer_module.cpp:59`; `FunctionServos.hpp:50`). If no UAVCAN_SVx_FUNC is a Servo, control[] never read but bridge still broadcasts (disarmed values for unset functions). Whole stack additionally requires UAVCAN_ENABLE>0 to start the node.

**Redundancy:** No per-source-node/multi-instance (OUTGOING command bridge, single broadcaster). One UavcanServoController + one UavcanMixingInterfaceServo per node (`uavcan_main.hpp:272-273`). One ArrayCommand broadcast to bus (`servo.cpp:60` `_uavcan_pub_array_cmd.broadcast(msg)`) with all enabled channels; remote nodes self-select by actuator_id. Servo and ESC are separate MixingOutput WorkItems (UAVCAN_SV vs UAVCAN_EC). No channel allocation across nodes (that applies only to incoming sensor bridges).

**Device-id encoding:** N/A. Outgoing actuator command path publishes no uORB sensor topic; no uORB device_id constructed. (The inverse-direction actuator_outputs published by MixingOutput at `mixer_module.cpp:529` is a separate generic mixer artifact, not this bridge.)

**Notes:**
- Transfer/scheduling: ArrayCommand broadcast at UAVCAN_COMMAND_TRANSFER_PRIORITY=6 (`servo.hpp:50`; ctor `servo.cpp:44`). Max rate MAX_RATE_HZ=50Hz -> setMaxTopicUpdateRate(20000us) (`servo.hpp:49`; `uavcan_main.cpp:124`). SchedulingPolicy::Auto: Run() driven by actuator_servos updates (`uavcan_main.cpp:1146`).
- WorkItem name MODULE_NAME "-actuators-servo" on wq uavcan (`uavcan_main.hpp:160`).
- Param prefix UAVCAN_SV, 8 channels (`module.yaml:92-100`). Per-channel (i=1..8): UAVCAN_SV_FUNCi, UAVCAN_SV_DISi (0..1000 default 500), UAVCAN_SV_MINi (0..1000 default 0), UAVCAN_SV_MAXi (0..1000 default 1000), UAVCAN_SV_FAILi (0..1000), reverse mask UAVCAN_SV_REVi. Ranges 0..1000 (unitless), NOT 1000..2000 PWM us.
- Scaling identity (default params): MixingOutput [-1,1]->[0,1000] then servo.cpp x/500-1 -> [-1,1]. If user changes MIN/MAX/DIS, the /500-1 formula no longer yields exactly [-1,1] (e.g. MAX=1000,MIN=200: +1->1000->+1.0, -1->200->-0.6). servo.cpp:55 comment flags this TODO. **Port MUST reproduce BOTH stages exactly, not emit control[] directly.**
- command_value wire type float16 (`Command.uavcan:21`) -> half-precision rounding; encode float16, not float32.
- ArrayCommand.commands capacity <=15; current code pushes up to 8; within bounds.
- Disarmed handling in MixingOutput (NaN->_disarmed_value, lockdown/kill->disarmed, termination->failsafe) (`mixer_module.cpp:491-505/538-539`); servo.cpp does NO clamping/NaN handling and always pushes num_outputs commands.
- FunctionServos.defaultFailsafeValue = 0.f (`FunctionServos.hpp:64`), used when UAVCAN_SV_FAILi left default (UINT16_MAX).

---

## 18. Hardpoint

**Source files:**
- `src/drivers/uavcan/actuators/hardpoint.cpp:42,55,69`
- `src/drivers/uavcan/actuators/hardpoint.hpp:70,82,85,87`

**DroneCAN message:** `uavcan.equipment.hardpoint.Command` (DTID 1070; `dsdl/uavcan/equipment/hardpoint/1070.Command.uavcan`).

**uORB topic + direction:** `vehicle_command` (subscribed; `hardpoint.hpp:85`, ORB_ID(vehicle_command)). NO uORB publication. **Outgoing** (the task header's "command (out)" refers to the DSDL field name, not a uORB topic).

**Field mapping:**

| DroneCAN out / gate | source field/expr | transform |
|---|---|---|
| Command.hardpoint_id (uint8) | `vehicle_command.param1` (f32) | implicit C++ float->uint8 (`_cmd.hardpoint_id = param1`; truncation toward zero, no rounding/scaling, modulo-256). `hardpoint.cpp:77` |
| Command.command (uint16) | `vehicle_command.param2` (f32) | implicit float->uint16 (truncation toward zero, no scaling, modulo-65536). param2 carries gripper action: GRIPPER_ACTION_RELEASE=0, GRIPPER_ACTION_GRAB=1 (`VehicleCommand.msg:208-210`). DSDL: 0=release, 1+=hold/bitmask. `hardpoint.cpp:78` |
| [gate/filter] vehicle_command.command must equal VEHICLE_CMD_DO_GRIPPER (211) | `vehicle_command.command == VEHICLE_CMD_DO_GRIPPER` | filter only — other command IDs ignored; only param1/param2 of a DO_GRIPPER(211) update _cmd. 211 at `msg/versioned/VehicleCommand.msg:63`. `hardpoint.cpp:76` |

**Compile gate:** `CONFIG_UAVCAN_HARDPOINT_CONTROLLER` (`Kconfig:21-23`, "Include hardpoint controller", default y). Guards member decl (`uavcan_main.hpp:275-277`), ctor init (`uavcan_main.cpp:93-95`), init() call (`uavcan_main.cpp:547-554`). Source compiled via `CMakeLists.txt:165`.

**Runtime gate:** NONE. No UAVCAN_SUB_*/UAVCAN_PUB_* runtime param (grep of uavcan_params.c found nothing). Once compiled in, init() called unconditionally and the 10Hz timer always runs. The DroneCAN Command is only actually broadcast after at least one VEHICLE_CMD_DO_GRIPPER (otherwise _cmd is default-zero, still broadcast at 1Hz from startup).

**Redundancy:** None. Single global controller instance (`uavcan_main.hpp:276`). Single uavcan::Publisher broadcasting (no destination node id). No per-source-node channel allocation, no multi-instance, no hardpoint_id-based instancing on the PX4 side (hardpoint_id just a payload field copied from param1). Outgoing bridge, no source-node redundancy.

**Device-id encoding:** N/A — outgoing actuator bridge with no uORB publication, so no uORB device_id. On DroneCAN side the message is a broadcast Command (DTID 1070) from the local node id with no addressing/instancing beyond the payload hardpoint_id field.

**Notes:**
- Behavioral model (`hardpoint.cpp:69-90`): libuavcan periodic timer at MAX_UPDATE_RATE_HZ=10Hz (period 100ms; `hardpoint.hpp:67`, started `:61-64`). Each tick: (1) if vehicle_command updated, copy it; if command==VEHICLE_CMD_DO_GRIPPER set _cmd.hardpoint_id=param1, _cmd.command=param2, force _next_publish_time=0. (2) if `hrt_absolute_time() > _next_publish_time`, broadcast _cmd and set _next_publish_time = now + 1e6/PUBLISH_RATE_HZ.
- Publish cadence: PUBLISH_RATE_HZ=1 (`hardpoint.hpp:68`) => Command re-broadcast at 1Hz even with no new vehicle_command. _cmd is PERSISTENT STATE (`hardpoint.hpp:70`): last hardpoint_id/command keeps being re-sent at 1Hz. **Port MUST replicate this 1Hz keep-alive re-broadcast of the last command, not just publish on change.**
- Immediate-publish on new command: _next_publish_time=0 (`hardpoint.cpp:79`) guarantees the next 10Hz tick (<=100ms) broadcasts the fresh command instead of waiting 1s. Net latency receipt->bus = up to one 100ms period.
- Initial state: _cmd default-constructed => hardpoint_id=0, command=0. From the first tick (before any vehicle_command), a zero/zero Command (0=release) is broadcast at 1Hz. Match this (no suppression of pre-command default broadcast).
- Transfer priority: TransferPriority::MiddleLower in ctor (`hardpoint.cpp:47`). Preserve.
- Narrowing caveat: param1->hardpoint_id and param2->command are plain float->unsigned-int assignments with NO clamping/rounding (e.g. param1=2.9f -> 2; negative/out-of-range is impl-defined). Reference = bare truncation toward zero. Match exactly.
- vehicle_command provenance: param1 (f32, `VehicleCommand.msg:227`), param2 (f32, `:228`); command (uint32, `:234`); VEHICLE_CMD_DO_GRIPPER=211 (`:63`); GRIPPER_ACTION_RELEASE=0 / GRIPPER_ACTION_GRAB=1 (`:209-210`).
- A Status message exists (uavcan.equipment.hardpoint.Status, DTID 1071) but is NOT used — no subscriber, no uORB publication. Status.hpp included in `hardpoint.hpp:44` but unused. Do NOT port a Status path.
