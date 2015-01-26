#!/bin/bash

# creates a binary distribution 
# (run after build.sh and ant)

cd platform_osx
./distclean.sh
cd ..
cd ..
zip -r axoloti_demo.zip axoloti/Axoloti.sh axoloti/Axoloti.command axoloti/dist axoloti/doc axoloti/license.txt axoloti/README.md axoloti/manifest.mf axoloti/objects axoloti/patch axoloti/patches axoloti/platform_osx axoloti/Axoloti.bat axoloti/platform_win
