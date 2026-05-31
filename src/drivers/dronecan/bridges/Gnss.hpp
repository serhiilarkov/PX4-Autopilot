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

/**
 * @file Gnss.hpp
 *
 * DroneCAN <-> uORB GNSS bridge (libcanard port of src/drivers/uavcan/sensors/gnss).
 * Subscribes uavcan.equipment.gnss.Fix / Fix2 / Auxiliary and ardupilot.gnss.
 * RelPosHeading (+ MovingBaselineData -> gps_dump when gated), maps to sensor_gps
 * per ACCEPTANCE_SPEC §1, and drives the outgoing RTCM / MovingBaselineData uplink
 * from gps_inject_data. The field mapping and all quirks live in the platform-free
 * GnssDecode unit; this class owns the transport registration, the bridge state and
 * the PX4 glue.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/gps_dump.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/sensor_gps.h>

#include <ardupilot.gnss.MovingBaselineData.h>
#include <ardupilot.gnss.RelPosHeading.h>
#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.gnss.RTCMStream.h>

#include "../DronecanHandle.hpp"
#include "../DronecanRxRouter.hpp"
#include "../RxSubscriberBase.hpp"
#include "GnssDecode.hpp"
#include "SensorBridgeBase.hpp"

class Gnss : public SensorBridgeBase, public RxSubscriberBase
{
public:
	explicit Gnss(DronecanHandle &handle);
	~Gnss() override;

	/// Read the per-bridge runtime gates (DC_PUB_RTCM / DC_PUB_MBD / DC_SUB_MBD) and
	/// register the RX handlers with the node's router. Fix/Fix2/Auxiliary/RelPosHeading
	/// are always registered; the incoming MovingBaselineData->gps_dump handler only
	/// when DC_SUB_MBD is set.
	void init(DronecanRxRouter &router);

	/// RxSubscriberBase: dispatched by the router for every registered GNSS type.
	/// iface_id (0 = CAN1, 1 = CAN2) is encoded into the sensor_gps device id bus field.
	void handle(const CanardRxTransfer &transfer, uint8_t iface_id) override;

	/// Pump the outgoing RTCM / MovingBaselineData uplink from gps_inject_data. Called
	/// once per node Run().
	void update();

	void print_status() const;

private:
	void handle_fix(const CanardRxTransfer &transfer, uint8_t iface_id);
	void handle_fix2(const CanardRxTransfer &transfer, uint8_t iface_id);
	void handle_auxiliary(const CanardRxTransfer &transfer);
	void handle_relpos(const CanardRxTransfer &transfer);
	void handle_moving_baseline_data(const CanardRxTransfer &transfer);

	/// process_fixx (gnss.cpp:396-560): finish the per-message mapping with the
	/// stateful/platform parts (device id, hrt timestamp, rel-heading consumption,
	/// CLOCK_REALTIME side effect, rtcm fields) and publish.
	template <typename FixT>
	void process_and_publish(const FixT &msg, int node_id, uint8_t iface_id, uint8_t fix_type,
				 const float (&pos_cov)[9], const float (&vel_cov)[9],
				 bool valid_pos_cov, bool valid_vel_cov,
				 float heading, float heading_offset, float heading_accuracy,
				 int32_t noise_per_ms, int32_t jamming_indicator,
				 uint8_t jamming_state, uint8_t spoofing_state);

	void handleInjectDataTopic();
	bool publishRTCMStream(const uint8_t *data, size_t data_len);
	bool publishMovingBaselineData(const uint8_t *data, size_t data_len);

	DronecanHandle &_handle;

	// One RxHandler node per subscribed type; intrusive list members owned here so
	// registration never mallocs (DronecanRxRouter stores the pointers).
	RxHandler _h_fix {};
	RxHandler _h_fix2 {};
	RxHandler _h_auxiliary {};
	RxHandler _h_relpos {};
	RxHandler _h_moving_baseline {};

	// Cached Auxiliary dop (global, last-writer-wins across nodes; gnss.cpp:142-144).
	uint64_t _last_gnss_auxiliary_timestamp{0};
	float    _last_gnss_auxiliary_hdop{0.0f};
	float    _last_gnss_auxiliary_vdop{0.0f};

	// RelPosHeading state (global, consumed once per fix; gnss.cpp:159-161).
	float _rel_heading_accuracy{NAN};
	float _rel_heading{NAN};
	bool  _rel_heading_valid{false};

	// Per-channel Fix-vs-Fix2 dedup: once a node sends Fix2 its Fix messages are dropped.
	bool _channel_using_fix2[DEFAULT_MAX_CHANNELS] {};

	bool _system_clock_set{false};

	// Outgoing uplink config + state.
	bool _publish_rtcm_stream{false};
	bool _publish_moving_baseline_data{false};
	bool _subscribe_moving_baseline_data{false};

	uORB::SubscriptionMultiArray<gps_inject_data_s, gps_inject_data_s::MAX_INSTANCES>
	_orb_inject_data_sub{ORB_ID::gps_inject_data};
	uORB::Publication<gps_dump_s> _gps_dump_pub{ORB_ID(gps_dump)};

	hrt_abstime _last_rtcm_injection_time{0};
	uint8_t     _selected_rtcm_instance{0};
	hrt_abstime _last_rate_measurement{0};
	float       _rtcm_injection_rate{0.f};
	unsigned    _rtcm_injection_rate_message_count{0};

	// Persistent per-publisher transfer ids + TX scratch buffers (>= each type max size).
	uint8_t _rtcm_transfer_id{0};
	uint8_t _moving_baseline_transfer_id{0};
	uint8_t _rtcm_buf[UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_MAX_SIZE] {};
	uint8_t _moving_baseline_buf[ARDUPILOT_GNSS_MOVINGBASELINEDATA_MAX_SIZE] {};
};
