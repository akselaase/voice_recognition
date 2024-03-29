#!/bin/bash

err=0
error() {
    err=$(( $err | $(( 2 << $1 )) ))
    echo $2
}

if [[ -z "$VIRTUAL_ENV" && -f "../venv/bin/activate" ]]; then
	. ../venv/bin/activate
fi

PYTHON="$(which python3 2>/dev/null)"
if [ -z "$PYTHON" ]; then
    echo "Didn't find Python 3, aborting."
	exit 1
fi
echo Found $($PYTHON --version)
if ! python3 -c "import sys; sys.exit(1 if sys.version_info[1] < 6 else 0)"; then
	echo "Warning: this package is not tested with python < 3.6."
fi

# Test python modules

$PYTHON -c "import yaml" >/dev/null 2>&1 && echo "Found yaml" || error 0 "Missing package pyyaml"
$PYTHON -c "import rospkg" >/dev/null 2>&1 && echo "Found rospkg" || error 0 "Missing package rospkg"
$PYTHON -c "import rospy" >/dev/null 2>&1 && echo "Found rospy" || error 0 "Missing package rospy"
$PYTHON -c "import std_msgs" >/dev/null 2>&1 && echo "Found std_msgs" || error 0 "Missing package std_msgs"
$PYTHON -c "import tensorflow" >/dev/null 2>&1 && echo "Found tensorflow" || error 0 "Missing package tensorflow(-gpu)"
if [ $err -ne 0 ]; then
	echo "Make sure to install the packages tensorflow-gpu, rospy, rospkg, and pyyaml."
fi
# Test binaries

which sox >/dev/null 2>&1 && echo "Found sox" || error 1 "Missing sox"

# Test local IP address retrieval
IP="$(ip route get 1 | awk '{print $7;exit}')"
[[ $IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "Successfully parsed local IP" || error 2 "Failed to retrieve local IP,
    you may need to manually edit ROS_IP in bin/run.sh"

if [ "$err" -eq "0" ]; then
    echo "All good!"
else
    echo "Exiting with code $err"
fi
exit $err
