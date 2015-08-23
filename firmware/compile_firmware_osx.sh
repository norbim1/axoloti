#!/bin/sh
export PATH=$PATH:${axoloti_runtime}/platform_osx/bin

echo "Compiling firmware... ${axoloti_firmware}"
cd "${axoloti_firmware}"
make -f Makefile.patch clean

mkdir -p build/obj
mkdir -p build/lst
if ! make ; then
    exit 1
fi

echo "Compiling firmware flasher..."
cd flasher
mkdir -p flasher_build/obj
mkdir -p flasher_build/lst
make
