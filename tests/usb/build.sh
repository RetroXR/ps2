#!/bin/sh
# GunCon 2 coordinate, calibration, parameter, and report protocol tests.
set -eu
DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOT=$(CDPATH= cd -- "$DIR/../.." && pwd)
SANFLAGS=""
[ -n "${SANITIZER:-}" ] && SANFLAGS="-fsanitize=$SANITIZER"

CXX=${CXX:-c++}
OUT="$DIR/guncon2_protocol"
case "$(uname -s 2>/dev/null)" in
	MINGW*|MSYS*|CYGWIN*) OUT="$OUT.exe" ;;
esac

"$CXX" -std=c++17 -O2 -Wall -Wextra -Werror -pthread $SANFLAGS \
	-I "$ROOT" -I "$ROOT/pcsx2" -I "$ROOT/common" -I "$ROOT/common/include" \
	-I "$ROOT/libretro" -I "$ROOT/libretro/libretro-common/include" \
	-o "$OUT" \
	"$DIR/guncon2_protocol.cpp" \
	"$ROOT/pcsx2/USB/libretro-usb/usb-guncon2-protocol.cpp" \
	"$ROOT/pcsx2/USB/libretro-usb/usb-guncon2-input.cpp"

"$OUT"
