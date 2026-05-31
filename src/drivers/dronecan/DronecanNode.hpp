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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#include "DronecanHandle.hpp"
#include "DronecanRxRouter.hpp"

class Gnss;

#if defined(DRONECAN_FDCAN_DRIVER)
# include "CanardFdcanIface.hpp"
#elif defined(CONFIG_NET_CAN)
# include "CanardSocketCAN.hpp"
#elif defined(CONFIG_CAN_EXTID)
# include "CanardNuttXCDev.hpp"
#endif

/**
 * DroneCAN (UAVCANv0 / libcanard) node.
 *
 * Singleton ScheduledWorkItem on wq:uavcan. Owns the platform-free transport
 * (DronecanHandle + DronecanRxRouter) and the concrete CAN media backend, wrapping
 * them in the PX4 glue (perf, hrt, logging, params). The CAN bring-up is lazy --
 * done on the first Run() on the WQ thread, never in the ctor -- and the node
 * pumps transmit/receive/transmit under _node_mutex each tick. See
 * dronecan_migration/DESIGN.md §6/§8.
 */
class DronecanNode : public ModuleParams, public px4::ScheduledWorkItem
{
	/*
	 * Base tick. Bounds TX latency, periodic node messages and stale-transfer
	 * cleanup; complemented by a CAN-RX-IRQ ScheduleNow() once a media backend can
	 * signal RX readiness (busevent_signal_trampoline).
	 */
	static constexpr unsigned ScheduleIntervalMs = 3;

public:
	DronecanNode(uint32_t node_id, uint32_t bitrate);
	~DronecanNode() override;

	static int start(uint32_t node_id, uint32_t bitrate);

	static DronecanNode *instance() { return _instance; }

	/// One-time, idempotent UAVCAN_* -> DC_* parameter migration. Static so the rcS
	/// board-init hook can run it before the DC_ENABLE start gate is evaluated (the
	/// cutover chicken-and-egg fix, DESIGN §7).
	static void migrateLegacyParams();

	void print_info();

private:
	void Run() override;

	/// Lazy bring-up on the WQ thread: open the CAN media, canardInit over the pool,
	/// assign the validated node id. Retried each Run() until the media is ready.
	bool init();

	/// CAN-RX-IRQ trampoline: only wakes the work queue (IRQ context, no canard
	/// access). The seam for the dual-scheduling model in DESIGN §6.
	static void busevent_signal_trampoline();

	/// Publish uavcan.protocol.NodeStatus at 1 Hz -- the first real publisher through
	/// the codec shim.
	void sendNodeStatus();

	static DronecanNode *_instance;

	uint32_t _node_id{0};
	uint32_t _bitrate{0};
	bool _initialized{false};

	pthread_mutex_t _node_mutex;

	// Transport stack. Declaration order matters: _handle holds references to
	// _rx_router and _can_iface, so they must be constructed first.
	DronecanRxRouter _rx_router;
#if defined(DRONECAN_FDCAN_DRIVER)
	CanardFdcanIface _can_iface;
#elif defined(CONFIG_NET_CAN)
	CanardSocketCAN _can_iface;
#elif defined(CONFIG_CAN_EXTID)
	CanardNuttXCDev _can_iface;
#else
	NullCanardInterface _can_iface;
#endif
	DronecanHandle _handle {_can_iface, _rx_router};

	// Sensor bridges. Heap-allocated in the lazy init() (gated by their DC_SUB_*),
	// registered with _rx_router; deleted in the dtor before _handle/_rx_router go.
	Gnss *_gnss{nullptr};

	// NodeStatus @ 1 Hz.
	hrt_abstime _boot_time{0};
	hrt_abstime _nodestatus_last{0};
	uint8_t _nodestatus_buf[7] {};   // sized to UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE (asserted in .cpp)
	uint8_t _nodestatus_transfer_id{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DC_ENABLE>) _param_dc_enable,
		(ParamInt<px4::params::DC_NODE_ID>) _param_dc_node_id,
		(ParamInt<px4::params::DC_BITRATE>) _param_dc_bitrate
	)
};
