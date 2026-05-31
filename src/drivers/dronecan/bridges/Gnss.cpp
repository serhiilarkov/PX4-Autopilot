/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Gnss.hpp"

#include "../DroneCANCodec.hpp"

#include <drivers/drv_sensor.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <cmath>
#include <cstring>
#include <time.h>

using namespace time_literals;

namespace
{

// Thin typed decode wrappers (the generated T -> decode map). The typed call lives
// here; the codec shim sees only the type-erased pointer and applies the single
// TRUE-on-failure inversion.
bool decode_fix(const CanardRxTransfer *t, void *m)
{
	return uavcan_equipment_gnss_Fix_decode(t, static_cast<uavcan_equipment_gnss_Fix *>(m));
}

bool decode_fix2(const CanardRxTransfer *t, void *m)
{
	return uavcan_equipment_gnss_Fix2_decode(t, static_cast<uavcan_equipment_gnss_Fix2 *>(m));
}

bool decode_auxiliary(const CanardRxTransfer *t, void *m)
{
	return uavcan_equipment_gnss_Auxiliary_decode(t, static_cast<uavcan_equipment_gnss_Auxiliary *>(m));
}

bool decode_relpos(const CanardRxTransfer *t, void *m)
{
	return ardupilot_gnss_RelPosHeading_decode(t, static_cast<ardupilot_gnss_RelPosHeading *>(m));
}

bool decode_moving_baseline(const CanardRxTransfer *t, void *m)
{
	return ardupilot_gnss_MovingBaselineData_decode(t, static_cast<ardupilot_gnss_MovingBaselineData *>(m));
}

uint32_t encode_rtcm(void *m, uint8_t *buf, bool tao)
{
	return uavcan_equipment_gnss_RTCMStream_encode(static_cast<uavcan_equipment_gnss_RTCMStream *>(m), buf, tao);
}

uint32_t encode_moving_baseline(void *m, uint8_t *buf, bool tao)
{
	return ardupilot_gnss_MovingBaselineData_encode(static_cast<ardupilot_gnss_MovingBaselineData *>(m), buf, tao);
}

// Outgoing type descriptors (data-as-trait seam). RTCM/MBD broadcast at the lowest
// priority (libuavcan TransferPriority::NumericallyMax == CANARD lowest).
const TypeDescriptor RtcmStreamDesc = {
	UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID,
	UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE,
	UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_MAX_SIZE,
	CanardTransferTypeBroadcast,
	"uavcan.equipment.gnss.RTCMStream"
};

const TypeDescriptor MovingBaselineDataDesc = {
	ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID,
	ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE,
	ARDUPILOT_GNSS_MOVINGBASELINEDATA_MAX_SIZE,
	CanardTransferTypeBroadcast,
	"ardupilot.gnss.MovingBaselineData"
};

} // namespace

Gnss::Gnss(DronecanHandle &handle) :
	SensorBridgeBase("dronecan_gnss", ORB_ID(sensor_gps)),
	_handle(handle)
{
}

Gnss::~Gnss()
{
}

void Gnss::init(DronecanRxRouter &router)
{
	int32_t pub_rtcm = 0;
	param_get(param_find("DC_PUB_RTCM"), &pub_rtcm);
	_publish_rtcm_stream = (pub_rtcm == 1);

	int32_t pub_mbd = 0;
	param_get(param_find("DC_PUB_MBD"), &pub_mbd);
	_publish_moving_baseline_data = (pub_mbd == 1);

	int32_t sub_mbd = 0;
	param_get(param_find("DC_SUB_MBD"), &sub_mbd);
	_subscribe_moving_baseline_data = (sub_mbd == 1);

	// Always-on incoming subscriptions.
	_h_fix = {UAVCAN_EQUIPMENT_GNSS_FIX_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_FIX_SIGNATURE, this, nullptr};
	_h_fix2 = {UAVCAN_EQUIPMENT_GNSS_FIX2_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE, this, nullptr};
	_h_auxiliary = {UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID, CanardTransferTypeBroadcast, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE, this, nullptr};
	_h_relpos = {ARDUPILOT_GNSS_RELPOSHEADING_ID, CanardTransferTypeBroadcast, ARDUPILOT_GNSS_RELPOSHEADING_SIGNATURE, this, nullptr};

	router.add(&_h_fix);
	router.add(&_h_fix2);
	router.add(&_h_auxiliary);
	router.add(&_h_relpos);

	// Incoming MovingBaselineData -> gps_dump (PPK) only when DC_SUB_MBD is set.
	if (_subscribe_moving_baseline_data) {
		_h_moving_baseline = {ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID, CanardTransferTypeBroadcast, ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE, this, nullptr};
		router.add(&_h_moving_baseline);
	}
}

void Gnss::handle(const CanardRxTransfer &transfer, uint8_t iface_id)
{
	switch (transfer.data_type_id) {
	case UAVCAN_EQUIPMENT_GNSS_FIX_ID:
		handle_fix(transfer, iface_id);
		break;

	case UAVCAN_EQUIPMENT_GNSS_FIX2_ID:
		handle_fix2(transfer, iface_id);
		break;

	case UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID:
		handle_auxiliary(transfer);
		break;

	case ARDUPILOT_GNSS_RELPOSHEADING_ID:
		handle_relpos(transfer);
		break;

	case ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID:
		handle_moving_baseline_data(transfer);
		break;

	default:
		break;
	}
}

void Gnss::handle_fix(const CanardRxTransfer &transfer, uint8_t iface_id)
{
	uavcan_equipment_gnss_Fix msg {};

	if (!dronecan_codec::decodeTransfer(transfer, &msg, decode_fix)) {
		return;
	}

	const int node_id = transfer.source_node_id;

	// Ignore the deprecated Fix message if this node also publishes Fix2.
	const int8_t ch = get_channel_index_for_node(node_id);

	if (ch > -1 && _channel_using_fix2[ch]) {
		return;
	}

	const uint8_t fix_type = msg.status;

	const bool valid_pos_cov = (msg.position_covariance.len != 0);
	const bool valid_vel_cov = (msg.velocity_covariance.len != 0);

	float pos_cov[9];
	dronecan_gnss::unpack_square_matrix_3x3(msg.position_covariance.data, msg.position_covariance.len, pos_cov);

	float vel_cov[9];
	dronecan_gnss::unpack_square_matrix_3x3(msg.velocity_covariance.data, msg.velocity_covariance.len, vel_cov);

	process_and_publish(msg, node_id, iface_id, fix_type, pos_cov, vel_cov, valid_pos_cov, valid_vel_cov,
			    NAN, NAN, NAN, -1, -1, 0, 0);
}

void Gnss::handle_fix2(const CanardRxTransfer &transfer, uint8_t iface_id)
{
	uavcan_equipment_gnss_Fix2 msg {};

	if (!dronecan_codec::decodeTransfer(transfer, &msg, decode_fix2)) {
		return;
	}

	const int node_id = transfer.source_node_id;

	// Latch this node onto the Fix2 path so later Fix messages are dropped. The flag
	// latches only once a channel exists (get_channel_index_for_node == -1 until then).
	const int8_t ch = get_channel_index_for_node(node_id);

	if (ch > -1 && !_channel_using_fix2[ch]) {
		PX4_DEBUG("GNSS Fix2 on ch %d; disabling Fix for this node", ch);
		_channel_using_fix2[ch] = true;
	}

	const uint8_t fix_type = dronecan_gnss::fix2_remap_fix_type(msg.status, msg.mode, msg.sub_mode);

	float pos_cov[9];
	float vel_cov[9];
	const bool valid_covariances = dronecan_gnss::fix2_unpack_covariance(msg, pos_cov, vel_cov);

	float heading = NAN;
	float heading_offset = NAN;
	float heading_accuracy = NAN;

	int32_t noise_per_ms = -1;
	int32_t jamming_indicator = -1;
	uint8_t jamming_state = 0;
	uint8_t spoofing_state = 0;

	// HACK (gnss.cpp:327-347): heading/noise/jamming smuggled through ecef_position_velocity
	// when RelPosHeading is not providing heading. Reproduced verbatim, including the
	// int64 position_xyz_mm -> int32 narrowing.
	if (msg.ecef_position_velocity.len != 0 && !_rel_heading_valid) {
		const auto &ecef = msg.ecef_position_velocity.data[0];

		if (!std::isnan(ecef.velocity_xyz[0])) {
			heading = ecef.velocity_xyz[0];
		}

		if (!std::isnan(ecef.velocity_xyz[1])) {
			heading_offset = ecef.velocity_xyz[1];
		}

		if (!std::isnan(ecef.velocity_xyz[2])) {
			heading_accuracy = ecef.velocity_xyz[2];
		}

		noise_per_ms = ecef.position_xyz_mm[0];
		jamming_indicator = ecef.position_xyz_mm[1];

		jamming_state = ecef.position_xyz_mm[2] >> 8;
		spoofing_state = ecef.position_xyz_mm[2] & 0xFF;
	}

	process_and_publish(msg, node_id, iface_id, fix_type, pos_cov, vel_cov, valid_covariances, valid_covariances,
			    heading, heading_offset, heading_accuracy, noise_per_ms, jamming_indicator,
			    jamming_state, spoofing_state);
}

void Gnss::handle_auxiliary(const CanardRxTransfer &transfer)
{
	uavcan_equipment_gnss_Auxiliary msg {};

	if (!dronecan_codec::decodeTransfer(transfer, &msg, decode_auxiliary)) {
		return;
	}

	// Store latest hdop/vdop for process_and_publish (global, last-writer-wins).
	_last_gnss_auxiliary_timestamp = hrt_absolute_time();
	_last_gnss_auxiliary_hdop = msg.hdop;
	_last_gnss_auxiliary_vdop = msg.vdop;
}

void Gnss::handle_relpos(const CanardRxTransfer &transfer)
{
	ardupilot_gnss_RelPosHeading msg {};

	if (!dronecan_codec::decodeTransfer(transfer, &msg, decode_relpos)) {
		return;
	}

	_rel_heading_valid = msg.reported_heading_acc_available;
	_rel_heading = math::radians(msg.reported_heading_deg);
	_rel_heading_accuracy = math::radians(msg.reported_heading_acc_deg);
}

void Gnss::handle_moving_baseline_data(const CanardRxTransfer &transfer)
{
	ardupilot_gnss_MovingBaselineData msg {};

	if (!dronecan_codec::decodeTransfer(transfer, &msg, decode_moving_baseline)) {
		return;
	}

	const int node_id = transfer.source_node_id;
	const size_t total_bytes = msg.data.len;
	size_t written = 0;

	while (written < total_bytes) {
		gps_dump_s dump = {};
		dump.timestamp = hrt_absolute_time();

		// device_id hand-built with bus=0 (NOT iface) and address=node_id&0xFF (gnss.cpp:374-378).
		dump.device_id = dronecan_bridge::make_uavcan_device_id((uint8_t)(node_id & 0xFF), 0, DRV_GPS_DEVTYPE_UAVCAN);

		dump.instance = 0; // Startup order of CAN nodes is non-deterministic.

		const size_t remaining = total_bytes - written;
		const size_t write_len = remaining < sizeof(dump.data) ? remaining : sizeof(dump.data);

		memcpy(dump.data, msg.data.data + written, write_len);
		dump.len = write_len;
		dump.len &= 0x7F; // MSB of len is direction -- 0 is from the device.

		written += write_len;

		_gps_dump_pub.publish(dump);
	}
}

template <typename FixT>
void Gnss::process_and_publish(const FixT &msg, const int node_id, const uint8_t iface_id, const uint8_t fix_type,
			       const float (&pos_cov)[9], const float (&vel_cov)[9],
			       const bool valid_pos_cov, const bool valid_vel_cov,
			       const float heading, const float heading_offset, const float heading_accuracy,
			       const int32_t noise_per_ms, const int32_t jamming_indicator,
			       const uint8_t jamming_state, const uint8_t spoofing_state)
{
	sensor_gps_s sensor_gps {};

	// device id bus field = the CAN interface the transfer arrived on (0 = CAN1, 1 = CAN2),
	// threaded down from the dispatch path (CanardRxTransfer itself carries no iface).
	sensor_gps.device_id = dronecan_bridge::make_uavcan_device_id((uint8_t)node_id, iface_id, DRV_GPS_DEVTYPE_UAVCAN);

	// NodeInfoPublisher::registerDeviceCapability is wired in P4.6; no-op until then.

	// FIXME HACK (gnss.cpp:417-425): publish time, not message time.
	sensor_gps.timestamp = hrt_absolute_time();

	// Resolve the heading triple: a valid RelPosHeading wins and is consumed once;
	// otherwise the caller's (Fix2 ecef-hack / Fix NAN) values are used (gnss.cpp:536-549).
	float h = heading;
	float off = heading_offset;
	float acc = heading_accuracy;

	if (_rel_heading_valid) {
		h = _rel_heading;
		off = NAN;
		acc = _rel_heading_accuracy;

		_rel_heading = NAN;
		_rel_heading_accuracy = NAN;
		_rel_heading_valid = false;
	}

	const bool aux_fresh = dronecan_gnss::aux_is_fresh(hrt_absolute_time(), _last_gnss_auxiliary_timestamp);

	dronecan_gnss::map_common(msg, fix_type, pos_cov, vel_cov, valid_pos_cov, valid_vel_cov,
				  h, off, acc, noise_per_ms, jamming_indicator, jamming_state, spoofing_state,
				  aux_fresh, _last_gnss_auxiliary_hdop, _last_gnss_auxiliary_vdop, sensor_gps);

	// Set the system clock once from GNSS time (gnss.cpp:509-521).
	if (sensor_gps.time_utc_usec != 0 && (fix_type >= sensor_gps_s::FIX_TYPE_2D) && !_system_clock_set) {
		timespec ts {};
		ts.tv_sec = sensor_gps.time_utc_usec / 1000000ULL;
		ts.tv_nsec = (sensor_gps.time_utc_usec % 1000000ULL) * 1000;
		px4_clock_settime(CLOCK_REALTIME, &ts);
		_system_clock_set = true;
	}

	sensor_gps.selected_rtcm_instance = _selected_rtcm_instance;
	sensor_gps.rtcm_injection_rate = _rtcm_injection_rate;

	publish(node_id, &sensor_gps);
}

void Gnss::update()
{
	handleInjectDataTopic();
}

void Gnss::handleInjectDataTopic()
{
	if (!_publish_rtcm_stream && !_publish_moving_baseline_data) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Measure RTCM update rate every 5 s.
	if (now > _last_rate_measurement + 5_s) {
		const float dt = (now - _last_rate_measurement) / 1e6f;
		_rtcm_injection_rate = _rtcm_injection_rate_message_count / dt;
		_last_rate_measurement = now;
		_rtcm_injection_rate_message_count = 0;
	}

	bool already_copied = false;
	gps_inject_data_s msg;

	// If there has not been a valid RTCM message for a while, try a different RTCM link.
	if (now > _last_rtcm_injection_time + 5_s) {
		for (int instance = 0; instance < _orb_inject_data_sub.size(); instance++) {
			const bool exists = _orb_inject_data_sub[instance].advertised();

			if (exists) {
				if (_orb_inject_data_sub[instance].copy(&msg)) {
					if ((hrt_absolute_time() - msg.timestamp) < 5_s) {
						already_copied = true;
						_selected_rtcm_instance = instance;
						break;
					}
				}
			}
		}
	}

	bool updated = already_copied;

	// Limit the number of injections; a full set is at most a few packets.
	const size_t max_num_injections = gps_inject_data_s::ORB_QUEUE_LENGTH;
	size_t num_injections = 0;

	do {
		if (updated) {
			num_injections++;

			if (_publish_rtcm_stream) {
				publishRTCMStream(msg.data, msg.len);
			}

			if (_publish_moving_baseline_data) {
				publishMovingBaselineData(msg.data, msg.len);
			}

			++_rtcm_injection_rate_message_count;
			_last_rtcm_injection_time = hrt_absolute_time();
		}

		auto &gps_inject_data_sub = _orb_inject_data_sub[_selected_rtcm_instance];
		updated = gps_inject_data_sub.update(&msg);

	} while (updated && num_injections < max_num_injections);
}

bool Gnss::publishRTCMStream(const uint8_t *const data, const size_t data_len)
{
	uavcan_equipment_gnss_RTCMStream msg {};
	const size_t capacity = sizeof(msg.data.data);
	size_t written = 0;
	bool result = true;

	while (result && written < data_len) {
		msg = {};
		msg.protocol_id = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_RTCM3;

		size_t chunk_size = data_len - written;

		if (chunk_size > capacity) {
			chunk_size = capacity;
		}

		for (size_t i = 0; i < chunk_size; ++i) {
			msg.data.data[msg.data.len++] = data[written++];
		}

		const int32_t res = dronecan_codec::encodeBroadcast(_handle, RtcmStreamDesc, &msg, _rtcm_buf,
				    &_rtcm_transfer_id, CANARD_TRANSFER_PRIORITY_LOWEST, encode_rtcm,
				    hrt_absolute_time() + 1_s);
		result = (res >= 0);
	}

	return result;
}

bool Gnss::publishMovingBaselineData(const uint8_t *data, size_t data_len)
{
	ardupilot_gnss_MovingBaselineData msg {};
	const size_t capacity = sizeof(msg.data.data);
	size_t written = 0;
	bool result = true;

	while (result && written < data_len) {
		msg = {};

		size_t chunk_size = data_len - written;

		if (chunk_size > capacity) {
			chunk_size = capacity;
		}

		for (size_t i = 0; i < chunk_size; ++i) {
			msg.data.data[msg.data.len++] = data[written++];
		}

		const int32_t res = dronecan_codec::encodeBroadcast(_handle, MovingBaselineDataDesc, &msg, _moving_baseline_buf,
				    &_moving_baseline_transfer_id, CANARD_TRANSFER_PRIORITY_LOWEST, encode_moving_baseline,
				    hrt_absolute_time() + 1_s);
		result = (res >= 0);
	}

	return result;
}

void Gnss::print_status() const
{
	SensorBridgeBase::print_status();
}
