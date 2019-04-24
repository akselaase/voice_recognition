#!/bin/bash

err=0
error() {
    err=$(( $err | $(( 1 << $1 )) ))
    echo $2
}

PYTHON="$(which python3 2>/dev/null)"
if [ -z "$PYTHON" ]; then
    echo "Didn't find Python 3, aborting."
fi
if ! python3 -c "import sys; sys.exit(1 if sys.version_info[1] < 6 else 0)"; then
	echo "Warning: this package is not tested with python < 3.6."
fi
echo Found $($PYTHON --version)

# Test python modules

$PYTHON -c "import rospy" >/dev/null 2>&1 && echo "Found rospy" || error 0 "Missing package rospy"
$PYTHON -c "import tensorflow" >/dev/null 2>&1 && echo "Found tensorflow" || error 1 "Missing package tensorflow"

# Test binaries

which sox >/dev/null 2>&1 && echo "Found sox" || error 2 "Missing sox"
which pv >/dev/null 2>&1 && echo "Found pv" || error 3 "Missing pv (pipeviewer)"

# Test local IP address retrieval
IP="$(ip route get 1 | awk '{print $7;exit}')"
[[ $IP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "Successfully parsed local IP" || error 4 "Failed to retrieve local IP,
    you may need to manually edit ROS_IP in bin/run.sh"

if [ "$err" -eq "0" ]; then
    echo "All good!"
else
    echo "Exiting with code $err"
fi
exit $err
