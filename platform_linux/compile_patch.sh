#!/bin/sh
platformdir="$(dirname $(readlink -f $0))"
export axoloti_release=${axoloti_release:="$platformdir/.."}
export axoloti_runtime=${axoloti_runtime:="$platformdir/.."}
export axoloti_firmware=${axoloti_firmware:="$axoloti_release/firmware"}
export axoloti_home=${axoloti_home:="$rootdir"}
export PATH=$PATH:${platformdir}/bin
cd ${axoloti_release}/patch
make
