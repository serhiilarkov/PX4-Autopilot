#!/usr/bin/env bash
#
# Off-target build + run of the DroneCAN GNSS bridge field-for-field contract test.
#
# Builds the platform-free GNSS mapping (bridges/GnssDecode) and the transport stack
# against the real libcanard canard.c + the generated GNSS codecs, plus the REAL
# generated uORB/topics/sensor_gps.h so the field map is asserted against the exact
# firmware struct. A tiny stubbed uORB/uORB.h (test/stub) satisfies that header
# without linking uORB.
#
# Unlike run_codec_test.sh this needs the generated sensor_gps.h, so a board must
# have been built once (e.g. `make ark_fmu-v6x`). Exits non-zero on any failed check.
#
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
drv="$(dirname "$here")"                          # src/drivers/dronecan
root="$(cd "$drv/../../.." && pwd)"               # PX4-Autopilot
gen="${TMPDIR:-/tmp}/dronecan_bridge_test/gen"
obj="${TMPDIR:-/tmp}/dronecan_bridge_test/obj"

# Locate a generated sensor_gps.h from any prior board build.
topic_inc=""
for d in "$root"/build/*/; do
	if [ -f "${d}uORB/topics/sensor_gps.h" ]; then
		topic_inc="${d%/}"
		break
	fi
done

if [ -z "$topic_inc" ]; then
	echo "ERROR: no build/<board>/uORB/topics/sensor_gps.h found." >&2
	echo "       Build a board first, e.g.: make ark_fmu-v6x" >&2
	exit 2
fi

# Generate the DSDL the test references (the generator emits the whole set).
python3 "$drv/dronecan_dsdlc/dronecan_dsdlc.py" -O "$gen" \
	"$drv/DSDL/uavcan" "$drv/DSDL/dronecan" "$drv/DSDL/ardupilot" \
	"$drv/DSDL/com" "$drv/DSDL/cuav" "$drv/DSDL/mppt" >/dev/null

flags="-DCANARD_ENABLE_TAO_OPTION=1 -DCANARD_ENABLE_CANFD=0 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_ALLOCATE_SEM=0"
# Stub uORB.h MUST be first so <uORB/uORB.h> resolves to it; the topic root provides
# <uORB/topics/sensor_gps.h>; src provides <lib/mathlib/...>.
inc="-I$here/stub -I$drv/libcanard -I$gen/include -I$topic_inc -I$root/src"

gnss_types="uavcan.equipment.gnss.Fix uavcan.equipment.gnss.Fix2 uavcan.equipment.gnss.Auxiliary ardupilot.gnss.RelPosHeading"

mkdir -p "$obj"
gcc -c $flags $inc "$drv/libcanard/canard.c" -o "$obj/canard.o"

for t in $gnss_types; do
	gcc -c $flags $inc "$gen/src/$t.c" -o "$obj/$t.o"
done

for f in DroneCANCodec DronecanRxRouter DronecanHandle; do
	g++ -std=gnu++14 -Wall -c $flags $inc "$drv/$f.cpp" -o "$obj/$f.o"
done

g++ -std=gnu++14 -Wall -c $flags $inc "$drv/bridges/GnssDecode.cpp"     -o "$obj/GnssDecode.o"
g++ -std=gnu++14 -Wall -c $flags $inc "$drv/test/dronecan_bridge_test.cpp" -o "$obj/test.o"
g++ "$obj"/*.o -o "$obj/dronecan_bridge_test"

exec "$obj/dronecan_bridge_test"
