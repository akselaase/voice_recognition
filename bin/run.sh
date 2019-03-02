#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
export ROS_IP=$(ip route get 1 | awk '{print $7;exit}')
export ROS_MASTER_URI=http://localhost:11311/
MODEL=arse_220k
#RECORDER="arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- -"
RECORDER="rec -q -r 16000 -b 16 -c 1 -e signed-integer --endian little -t wav -"
PYTHON2="python2.7"
PYTHON3="python3.6"
TEE="tee rec.wav"

if [[ ! -f "$PWD/run.sh" ]]; then 
    echo "This script should be run from the bin/ folder or with \"roslaunch voice_recognition main.launch\""
    exit 1
fi

if [[ ! -z "$1" && -f "$1" ]]; then
    RECORDER="eval pv -q -L 16000 < $1"
    TEE="cat"
else
    echo "Double check that the script is using the correct audio input device" \
         "by using the built in PulseAudio settings panel or pavucontrol"
fi

cat ../data/models/$MODEL/desc.txt
echo ---

$RECORDER | $TEE | \
    $PYTHON3 -u ../src/splitter/splitter.py -t 5.0,1.0 | \
    $PYTHON3 -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    $PYTHON3 -u ../src/filter/confidence_filter.py | \
    $PYTHON3 -u ../src/filter/chain_filter.py | \
    $PYTHON2 -u ../src/publisher/publisher.py
