#!/bin/bash

err=0
error() {
    err=$(( $err | $(( 1 << $1 )) ))
    echo $2
}

# Find python 2 and 3
PYTHON2="$(which python2.7)"
PYTHON3="$(which python3.6)"
if [ -z "$PYTHON2" ]; then
    echo "Didn't find Python 2.7, aborting"
    exit 1
fi
if [ -z "$PYTHON3" ]; then
    echo "Didn't find Python 3.6, aborting"
    exit 1
fi

echo Found $($PYTHON2 --version 2>&1) and $($PYTHON3 --version)

# Test python modules

$PYTHON2 -c "import rospy" >/dev/null 2>&1 && echo "Found rospy" || error 0 "Missing package rospy"
$PYTHON3 -c "import tensorflow" >/dev/null 2>&1 && echo "Found tensorflow" || error 1 "Missing package tensorflow"

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
