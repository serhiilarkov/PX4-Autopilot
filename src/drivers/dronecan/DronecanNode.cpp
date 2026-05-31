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

#include "DronecanNode.hpp"
#include "DroneCANCodec.hpp"

#include <lib/parameters/param.h>

#include <canard.h>
#include <uavcan.protocol.NodeStatus.h>

#include <inttypes.h>
#include <string.h>

using namespace time_literals;

DronecanNode *DronecanNode::_instance;

namespace
{

// DroneCAN unicast node IDs are 1..125 (libcanard allows up to 127; the spec and the
// DC_NODE_ID param cap at 125).
constexpr int32_t DroneCanMaxNodeID = 125;

bool is_unicast(int32_t node_id)
{
	return node_id >= CANARD_MIN_NODE_ID && node_id <= DroneCanMaxNodeID;
}

// NodeStatus codec seam -- the data-as-trait descriptor plus the thin typed encode
// wrapper the generated T -> {encode} map will emit (no function-pointer cast).
const TypeDescriptor NodeStatusDesc = {
	UAVCAN_PROTOCOL_NODESTATUS_ID,
	UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
	UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
	CanardTransferTypeBroadcast,
	"uavcan.protocol.NodeStatus"
};

uint32_t dc_nodestatus_encode(void *msg, uint8_t *buffer, bool tao)
{
	return uavcan_protocol_NodeStatus_encode(static_cast<uavcan_protocol_NodeStatus *>(msg), buffer, tao);
}

} // namespace

DronecanNode::DronecanNode(uint32_t node_id, uint32_t bitrate) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_node_id(node_id),
	_bitrate(bitrate)
{
	pthread_mutex_init(&_node_mutex, nullptr);
}

DronecanNode::~DronecanNode()
{
	ScheduleClear();

	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	pthread_mutex_destroy(&_node_mutex);

	_instance = nullptr;
}

int DronecanNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	_instance = new DronecanNode(node_id, bitrate);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

bool DronecanNode::init()
{
	// Lazy bring-up on the WQ thread (never the ctor): open the CAN media, init the
	// libcanard instance over the pool, assign the validated node id.
#if defined(DRONECAN_FDCAN_DRIVER)

	if (_can_iface.init(_bitrate) != 0) {
		// Hardware not ready yet; retry on the next tick.
		return false;
	}

	// Dual scheduling (DESIGN §6): the FDCAN driver's CAN-RX-IRQ BusEvent fires this
	// trampoline (ScheduleNow()), complementing the periodic 3 ms tick.
	_can_iface.registerBusEventCallback(&DronecanNode::busevent_signal_trampoline);

#else

	if (_can_iface.init() != 0) {
		// Media not ready, or no CAN backend configured on this board
		// (NullCanardInterface::init() always fails). Retry on the next tick.
		return false;
	}

#endif

	int32_t pool_bytes = DronecanHandle::PoolStorageBytes;
	param_get(param_find("DC_POOL"), &pool_bytes);
	_handle.init((uint32_t)pool_bytes);

	// Node id was validated isUnicast in start(); PX4 is the DroneCAN server, never an
	// anonymous DNA client, so no TX is issued before the id is set.
	_handle.setNodeID((uint8_t)_node_id);

	_boot_time = hrt_absolute_time();
	_initialized = true;

	PX4_INFO("DroneCAN node %" PRIu32 " up (%" PRIu32 " bit/s, pool %" PRId32 " B)",
		 _node_id, _bitrate, pool_bytes);

	return true;
}

void DronecanNode::busevent_signal_trampoline()
{
	if (_instance != nullptr) {
		// Called from CAN-RX-IRQ context: only kick the work queue, never touch canard.
		_instance->ScheduleNow();
	}
}

void DronecanNode::sendNodeStatus()
{
	static_assert(sizeof(_nodestatus_buf) == UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
		      "NodeStatus TX buffer must match the generated max size");

	const hrt_abstime now = hrt_absolute_time();

	if (now - _nodestatus_last < 1_s) {
		return;
	}

	uavcan_protocol_NodeStatus msg {};
	msg.uptime_sec = (uint32_t)((now - _boot_time) / 1_s);
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 0;

	const int32_t res = dronecan_codec::encodeBroadcast(
				    _handle, NodeStatusDesc, &msg, _nodestatus_buf, &_nodestatus_transfer_id,
				    CANARD_TRANSFER_PRIORITY_LOW, dc_nodestatus_encode, now + 1_s);

	if (res < 0) {
		// Pool exhaustion is already counted in the handle and surfaced via 'status'.
		PX4_DEBUG("NodeStatus enqueue failed (%" PRId32 ")", res);
	}

	_nodestatus_last = now;
}

void DronecanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	perf_count(_interval_perf);
	perf_begin(_cycle_perf);

	if (!_initialized) {
		if (!init()) {
			perf_end(_cycle_perf);
			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// Periodic node messages.
	sendNodeStatus();

	// transmit(); receive(); transmit(); -- the trailing transmit flushes service
	// responses that receive()'s onReception callbacks enqueue this same tick (DESIGN §6).
	_handle.transmit();
	_handle.receive();
	_handle.transmit();

	// Stale-transfer cleanup, once per pass.
	_handle.cleanupStale(hrt_absolute_time());

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);
}

void DronecanNode::migrateLegacyParams()
{
	// One-time, idempotent UAVCAN_* -> DC_* import. Runs from the rcS board-init hook
	// before the DC_ENABLE start gate (so a board reflashed from libuavcan inherits its
	// enable/node-id on first boot) and again, harmlessly, at 'dronecan start'.
	//
	// NB: the import only finds a value while the legacy UAVCAN_* definition is still
	// registered (DESIGN §7 keeps it for one transition release). On a board where the
	// uavcan module -- and thus its params -- are already excluded, each pair is a safe
	// no-op until that retention is wired.
	const param_t sentinel = param_find("DC_MIGRATED");
	int32_t done = 0;

	if (sentinel != PARAM_INVALID && param_get(sentinel, &done) == PX4_OK && done != 0) {
		return;
	}

	static const struct {
		const char *legacy;
		const char *current;
	} table[] = {
		{"UAVCAN_ENABLE",  "DC_ENABLE"},
		{"UAVCAN_NODE_ID", "DC_NODE_ID"},
		{"UAVCAN_BITRATE", "DC_BITRATE"},
	};

	for (const auto &pair : table) {
		const param_t legacy = param_find_no_notification(pair.legacy);
		const param_t current = param_find(pair.current);

		if (legacy == PARAM_INVALID || current == PARAM_INVALID) {
			continue;
		}

		// Non-destructive: a user who already set DC_* wins.
		if (!param_value_is_default(current)) {
			continue;
		}

		// All migrated params are int32 in this phase (DC_ENABLE/NODE_ID/BITRATE).
		int32_t value = 0;

		if (param_get(legacy, &value) == PX4_OK) {
			param_set_no_notification(current, &value);
		}
	}

	if (sentinel != PARAM_INVALID) {
		int32_t one = 1;
		param_set_no_notification(sentinel, &one);
	}
}

void DronecanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	PX4_INFO("DroneCAN node id %" PRIu32 ", bitrate %" PRIu32 " bit/s, %s",
		 _node_id, _bitrate, _initialized ? "on bus" : "waiting for CAN media");

	if (_initialized) {
		const CanardPoolAllocatorStatistics st = _handle.poolStats();
		PX4_INFO("pool: %u/%u blocks (peak %u), exhausted count %" PRIu32,
			 st.current_usage_blocks, st.capacity_blocks, st.peak_usage_blocks,
			 _handle.poolExhaustedCount());
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: dronecan {start|stop|status|migrate|shrink}");
}

extern "C" __EXPORT int dronecan_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	// Out-of-band param migration (rcS board-init hook), runnable without an instance.
	if (!strcmp(argv[1], "migrate")) {
		DronecanNode::migrateLegacyParams();
		return 0;
	}

	if (!strcmp(argv[1], "start")) {
		if (DronecanNode::instance() != nullptr) {
			PX4_ERR("already started");
			return 1;
		}

		// Import any legacy UAVCAN_* settings before reading DC_* (idempotent).
		DronecanNode::migrateLegacyParams();

		int32_t node_id = 0;
		param_get(param_find("DC_NODE_ID"), &node_id);

		int32_t bitrate = 0;
		param_get(param_find("DC_BITRATE"), &bitrate);

		if (!is_unicast(node_id)) {
			PX4_ERR("Invalid Node ID %" PRId32 " (must be %d..%" PRId32 ")",
				node_id, CANARD_MIN_NODE_ID, DroneCanMaxNodeID);
			return 1;
		}

		PX4_INFO("Node ID %" PRId32 ", bitrate %" PRId32, node_id, bitrate);
		return DronecanNode::start((uint32_t)node_id, (uint32_t)bitrate);
	}

	/* commands below require the app to be started */
	DronecanNode *const inst = DronecanNode::instance();

	if (inst == nullptr) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "shrink")) {
		// The pool is a fixed pre-allocated static buffer; there is nothing to free back.
		PX4_INFO("dronecan uses a fixed pre-allocated pool; shrink is a no-op");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
