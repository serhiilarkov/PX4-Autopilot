############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Legacy DroneCAN-v0 driver built on the DroneCAN fork of libcanard.
# The four vendored submodules are pinned to ArduPilot's commits.
set(DRONECAN_LIBCANARD_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/libcanard)
set(DRONECAN_DSDLC_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/dronecan_dsdlc)
set(DRONECAN_DSDL_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/DSDL)
set(DRONECAN_PYDRONECAN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pydronecan)

px4_add_git_submodule(TARGET git_dronecan_libcanard  PATH ${DRONECAN_LIBCANARD_DIR})
px4_add_git_submodule(TARGET git_dronecan_dsdlc      PATH ${DRONECAN_DSDLC_DIR})
px4_add_git_submodule(TARGET git_dronecan_dsdl       PATH ${DRONECAN_DSDL_DIR})
px4_add_git_submodule(TARGET git_dronecan_pydronecan PATH ${DRONECAN_PYDRONECAN_DIR})

# Belt-and-suspenders mutual exclusion. The legacy DroneCAN (this module) and the
# Cyphal/UAVCANv0 drivers each vendor their own libcanard generation; the two
# generations export colliding canard* symbols and must never co-link. Kconfig
# already enforces this (DRIVERS_CYPHAL/DRIVERS_UAVCAN depends on !DRIVERS_DRONECAN);
# fail loudly if a board config somehow set both.
if(CONFIG_DRIVERS_UAVCAN)
	message(FATAL_ERROR "DRIVERS_DRONECAN and DRIVERS_UAVCAN are mutually exclusive (canard* symbol collision); disable one")
endif()

if(CONFIG_DRIVERS_CYPHAL)
	message(FATAL_ERROR "DRIVERS_DRONECAN and DRIVERS_CYPHAL are mutually exclusive (canard* symbol collision); disable one")
endif()

# DSDL code generation. Configure-time on purpose (mirrors cyphal's nnvg
# execute_process): the generated *.c are globbed into SRCS below, so they must
# already exist when this file is processed -- a build-time custom command would
# glob an empty directory on a clean tree. Editing DSDL therefore requires a
# re-run of cmake. pydronecan is auto-located by the generator via the sibling
# ../pydronecan path.
set(DRONECAN_DSDLC_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated)

message(STATUS "Generating DroneCAN DSDL headers/sources with dronecan_dsdlc")
execute_process(
	COMMAND ${PYTHON_EXECUTABLE} ${DRONECAN_DSDLC_DIR}/dronecan_dsdlc.py
		-O ${DRONECAN_DSDLC_OUTPUT}
		${DRONECAN_DSDL_DIR}/uavcan
		${DRONECAN_DSDL_DIR}/dronecan
		${DRONECAN_DSDL_DIR}/ardupilot
		${DRONECAN_DSDL_DIR}/com
		${DRONECAN_DSDL_DIR}/cuav
		${DRONECAN_DSDL_DIR}/mppt
	RESULT_VARIABLE DRONECAN_DSDLC_RESULT
)

if(NOT DRONECAN_DSDLC_RESULT EQUAL 0)
	message(FATAL_ERROR "dronecan_dsdlc DSDL generation failed (rc=${DRONECAN_DSDLC_RESULT})")
endif()

file(GLOB DRONECAN_GENERATED_SRCS ${DRONECAN_DSDLC_OUTPUT}/src/*.c)

# Media backend selection. Preferred: the shared, framework-neutral STM32H7 FDCAN
# driver extracted into the platform layer (platforms/nuttx/.../stm32h7/can, written
# against the px4can shim -- no libuavcan dependency). Because dronecan and the legacy
# uavcan module are mutually exclusive at link time, each can compile the shared driver
# sources directly with no symbol conflict. The native-NuttX SocketCAN / CAN char-device
# backends are kept as future stubs; a board with neither falls back to the header-only
# NullCanardInterface in the node.
set(DRONECAN_MEDIA_SRCS)
set(DRONECAN_MEDIA_INCLUDES)
set(DRONECAN_MEDIA_FLAGS)

set(PX4_STM32H7_CAN_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/stm/stm32h7/can)

if(${PX4_PLATFORM} MATCHES "nuttx" AND "${CONFIG_ARCH_CHIP}" MATCHES "stm32h7" AND NOT CONFIG_NET_CAN)
	list(APPEND DRONECAN_MEDIA_SRCS
		CanardFdcanIface.cpp
		CanardFdcanIface.hpp
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_can.cpp
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_clock.cpp
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_thread.cpp
	)
	list(APPEND DRONECAN_MEDIA_INCLUDES
		${PX4_STM32H7_CAN_DIR}/include
	)
	list(APPEND DRONECAN_MEDIA_FLAGS
		-DDRONECAN_FDCAN_DRIVER=1
		# ark_fmu-v6x routes both CAN1 and CAN2 to transceivers (board_get_can_interfaces
		# == 0x3); run both, matching the legacy uavcan driver. The media fans RX/TX across
		# getNumIfaces() and tags each frame's source interface into the device id.
		-DUAVCAN_STM32H7_NUM_IFACES=2
	)
	# The extracted libuavcan-derived register driver predates PX4's C++ warning set.
	set_source_files_properties(
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_can.cpp
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_clock.cpp
		${PX4_STM32H7_CAN_DIR}/uc_stm32h7_thread.cpp
		CanardFdcanIface.cpp
		PROPERTIES COMPILE_FLAGS "-Wno-cast-align -Wno-deprecated-copy -Wno-address-of-packed-member"
	)
elseif(${PX4_PLATFORM} MATCHES "nuttx" AND CONFIG_NET_CAN)
	list(APPEND DRONECAN_MEDIA_SRCS
		CanardSocketCAN.cpp
		CanardSocketCAN.hpp
	)
elseif(${PX4_PLATFORM} MATCHES "nuttx" AND CONFIG_CAN_EXTID)
	list(APPEND DRONECAN_MEDIA_SRCS
		CanardNuttXCDev.cpp
		CanardNuttXCDev.hpp
	)
endif()

# Legacy libcanard + ArduPilot-generated C are not written for PX4's -Werror.
remove_definitions(-Werror)

px4_add_module(
	MODULE drivers__dronecan
	MAIN dronecan
	STACK_MAIN 4096
	COMPILE_FLAGS
		# libcanard build policy (DESIGN Q5): classic 8-byte CAN, deadline field
		# populated (40 B block), no internal semaphore. DRONECAN_CXX_WRAPPERS is
		# deliberately left undefined so the per-type cxx_iface trait is never emitted.
		-DCANARD_ENABLE_TAO_OPTION=1
		-DCANARD_ENABLE_CANFD=0
		-DCANARD_ENABLE_DEADLINE=1
		-DCANARD_ALLOCATE_SEM=0
		${DRONECAN_MEDIA_FLAGS}
	INCLUDES
		${DRONECAN_LIBCANARD_DIR}
		${DRONECAN_DSDLC_OUTPUT}/include
		${DRONECAN_MEDIA_INCLUDES}
	SRCS
		DronecanNode.cpp
		DronecanNode.hpp
		CanardInterface.hpp
		DroneCANCodec.cpp
		DroneCANCodec.hpp
		DronecanRxRouter.cpp
		DronecanRxRouter.hpp
		DronecanHandle.cpp
		DronecanHandle.hpp
		RxSubscriberBase.hpp
		bridges/DeviceId.hpp
		bridges/SensorBridgeBase.cpp
		bridges/SensorBridgeBase.hpp
		bridges/GnssDecode.cpp
		bridges/GnssDecode.hpp
		bridges/Gnss.cpp
		bridges/Gnss.hpp
		${DRONECAN_MEDIA_SRCS}
		${DRONECAN_LIBCANARD_DIR}/canard.c
		${DRONECAN_LIBCANARD_DIR}/canard.h
		${DRONECAN_GENERATED_SRCS}
	MODULE_CONFIG
		module.yaml
		dronecan.params.yaml
	DEPENDS
		git_dronecan_libcanard
		git_dronecan_dsdlc
		git_dronecan_dsdl
		git_dronecan_pydronecan
		version
	)

# libcanard triggers -Wcast-align; silence it on the library translation unit only
# (cyphal does the same on its canard.c).
set_source_files_properties(${DRONECAN_LIBCANARD_DIR}/canard.c PROPERTIES COMPILE_FLAGS -Wno-cast-align)
