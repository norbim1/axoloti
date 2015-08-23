#!/bin/sh
export PATH=$PATH:${axoloti_runtime}/platform_linux/bin

cd "${axoloti_firmware}"
make -f Makefile.patch clean

echo "Compiling firmware... ${axoloti_firmware}"
mkdir -p build/obj
mkdir -p build/lst
if ! make ; then
    exit 1
fi

echo "Compiling firmware flasher..."
cd flasher
mkdir -p flasher_build/lst
mkdir -p flasher_build/obj
make
