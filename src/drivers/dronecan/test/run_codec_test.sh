#!/usr/bin/env bash
#
# Off-target build + run of the DroneCAN transport contract test.
#
# Builds the pure transport stack (DroneCANCodec / DronecanRxRouter /
# DronecanHandle) against the real libcanard canard.c and the generated
# uavcan.protocol.NodeStatus codec, with the locked canard build flags, and runs
# the contract checks (decode inversion, encode->decode identity, wrong-length
# rejection, fake-frame dispatch through the full handle + router stack).
#
# No PX4 platform deps and no firmware build -- runs anywhere a host compiler is
# available. Exits non-zero on any failed check.
#
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
drv="$(dirname "$here")"                    # src/drivers/dronecan
gen="${TMPDIR:-/tmp}/dronecan_codec_test/gen"
obj="${TMPDIR:-/tmp}/dronecan_codec_test/obj"

# Generate just the DSDL the test references (the generator emits the whole set).
python3 "$drv/dronecan_dsdlc/dronecan_dsdlc.py" -O "$gen" \
	"$drv/DSDL/uavcan" "$drv/DSDL/dronecan" "$drv/DSDL/ardupilot" \
	"$drv/DSDL/com" "$drv/DSDL/cuav" "$drv/DSDL/mppt" >/dev/null

flags="-DCANARD_ENABLE_TAO_OPTION=1 -DCANARD_ENABLE_CANFD=0 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_ALLOCATE_SEM=0"
inc="-I$drv/libcanard -I$gen/include"

mkdir -p "$obj"
gcc -c $flags $inc "$drv/libcanard/canard.c"                 -o "$obj/canard.o"
gcc -c $flags $inc "$gen/src/uavcan.protocol.NodeStatus.c"   -o "$obj/NodeStatus.o"
for f in DroneCANCodec DronecanRxRouter DronecanHandle; do
	g++ -std=gnu++14 -Wall -c $flags $inc "$drv/$f.cpp"      -o "$obj/$f.o"
done
g++ -std=gnu++14 -Wall -c $flags $inc "$drv/test/dronecan_codec_test.cpp" -o "$obj/test.o"
g++ "$obj"/*.o -o "$obj/dronecan_codec_test"

exec "$obj/dronecan_codec_test"
