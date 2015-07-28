#!/bin/bash

platform='unknown'
unamestr=`uname`
case "$unamestr" in
	Linux)
		platform='linux'
		rootdir="$(dirname $(readlink -f $0))"
		export axoloti_release=${axoloti_release:="$rootdir"}
		export axoloti_runtime=${axoloti_runtime:="$rootdir"}
		export axoloti_firmware=${axoloti_firmware:="$axoloti_release/firmware"}
		export axoloti_home=${axoloti_home:="$rootdir"}
	;;
	Darwin)
		platform='mac'
		rootdir="$(cd $(dirname $0); pwd -P)"
		export axoloti_release=${axoloti_release:="$rootdir"}
		export axoloti_runtime=${axoloti_runtime:="$rootdir"}
		export axoloti_firmware=${axoloti_firmware:="$axoloti_release/firmware"}
		export axoloti_home=${axoloti_home:="$rootdir"}
	;;
        *)
                echo "unknown OS : $unamestr, aborting..."
                exit
        ;;
esac

which java >/dev/null || echo "java not found in path" 

if [ -f $rootdir/dist/Axoloti.jar ]
then
    case "$platform" in
        mac)
                java -Xdock:name=Axoloti -jar $rootdir/dist/Axoloti.jar
        ;;
        linux)
                java -jar $rootdir/dist/Axoloti.jar
        ;;
    esac
else
    echo "Axoloti.jar does not exist."
fi
