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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

/**
 * DroneCAN (UAVCANv0 / libcanard) node.
 *
 * P4.0/P4.1 scaffold: a singleton ScheduledWorkItem on wq:uavcan with a
 * start/stop/status CLI and the DC_* parameters. The transport (DronecanHandle),
 * the codec shim (DroneCANCodec) and the dispatch router exist (P4.1); the node
 * wires them on first Run() and the sensor/actuator bridges follow, starting in
 * P4.3 -- see dronecan_migration/DESIGN.md.
 */
class DronecanNode : public ModuleParams, public px4::ScheduledWorkItem
{
	/*
	 * Base interval; will be complemented by a CAN-RX-IRQ ScheduleNow() once the
	 * node wires the transport (P4.3), to decrease response time.
	 */
	static constexpr unsigned ScheduleIntervalMs = 3;

public:
	DronecanNode(uint32_t node_id, uint32_t bitrate);
	~DronecanNode() override;

	static int start(uint32_t node_id, uint32_t bitrate);

	static DronecanNode *instance() { return _instance; }

	void print_info();

private:
	void Run() override;

	static DronecanNode *_instance;

	uint32_t _node_id{0};
	uint32_t _bitrate{0};

	pthread_mutex_t _node_mutex;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DC_ENABLE>) _param_dc_enable,
		(ParamInt<px4::params::DC_NODE_ID>) _param_dc_node_id,
		(ParamInt<px4::params::DC_BITRATE>) _param_dc_bitrate
	)
};
