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

#include <stdint.h>

namespace dronecan_bridge
{

/// device::Device::DeviceBusType_UAVCAN. Hard-coded so this header stays
/// platform-free (host-testable) -- the value is frozen by the device-id wire/log
/// format, identical to device::Device.
static constexpr uint8_t DeviceBusType_UAVCAN = 3;

/**
 * Pack a PX4 device id for a DroneCAN node, byte-identical to
 * UavcanSensorBridgeBase::make_uavcan_device_id (REF sensor_bridge.hpp:161-169) and
 * to device::Device::DeviceId's little-endian bitfield layout:
 *   bits[0:2]   = bus_type (UAVCAN = 3)
 *   bits[3:7]   = bus      (iface_index: 0 = CAN1, 1 = CAN2)
 *   bits[8:15]  = address  (node_id)
 *   bits[16:23] = devtype  (sensor-class device type, e.g. 0x85 GPS)
 *
 * Example (ACCEPTANCE_SPEC §2): node 125 on CAN1, devtype 0x85 -> 0x00857D03.
 */
inline uint32_t make_uavcan_device_id(uint8_t node_id, uint8_t iface_index, uint8_t devtype)
{
	return (uint32_t)(DeviceBusType_UAVCAN & 0x7)
	       | ((uint32_t)(iface_index & 0x1F) << 3)
	       | ((uint32_t)node_id << 8)
	       | ((uint32_t)devtype << 16);
}

} // namespace dronecan_bridge
