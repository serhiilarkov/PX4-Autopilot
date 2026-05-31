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

#include <uORB/uORB.h>

#include <stdint.h>

#include "DeviceId.hpp"

/**
 * Base for redundant DroneCAN sensor bridges: one independent ORB instance per
 * source node id. Ports the UavcanSensorBridgeBase channel-allocation behavior
 * (REF sensor_bridge.{hpp,cpp}) -- up to DEFAULT_MAX_CHANNELS channels keyed by
 * node_id, allocated lazily via orb_advertise_multi on first sighting, with an
 * _out_of_channels latch that drops further new nodes.
 *
 * Unlike the libuavcan base this does NOT inherit device::Device: the per-channel
 * device id is not what bridges publish (ACCEPTANCE_SPEC §1 "device-id encoding"),
 * so each report's device_id is built per-message via make_uavcan_device_id().
 */
class SensorBridgeBase
{
public:
	static constexpr unsigned DEFAULT_MAX_CHANNELS = 4;

	virtual ~SensorBridgeBase();

	/// Number of source nodes that have been allocated a channel.
	unsigned get_num_redundant_channels() const;

	/// Channel index (0..max-1) currently bound to node_id, or -1 if none. Used by
	/// the GNSS Fix/Fix2 per-node dedup (the flag latches only once a channel exists).
	int8_t get_channel_index_for_node(int node_id) const;

	void print_status() const;

protected:
	struct Channel {
		int node_id{-1};
		orb_advert_t orb_advert{nullptr};
		int instance{-1};
	};

	SensorBridgeBase(const char *name, const orb_id_t orb_topic) :
		_name(name), _orb_topic(orb_topic) {}

	/**
	 * Publish one report for a source node, allocating a new ORB instance the first
	 * time that node is seen. Port of UavcanSensorBridgeBase::publish
	 * (sensor_bridge.cpp:256-317).
	 */
	void publish(int node_id, const void *report);

	const char     *_name;
	const orb_id_t  _orb_topic;
	Channel         _channels[DEFAULT_MAX_CHANNELS] {};
	bool            _out_of_channels{false};
};
