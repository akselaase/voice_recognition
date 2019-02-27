#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
export ROS_IP=$(ip route get 1 | awk '{print $7;exit}')
export ROS_MASTER_URI=http://localhost:11311/
export AUDIODEVICE=hw:CARD=UR22,DEV=0
MODEL=arse_220k
#RECORDER="arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- -"
RECORDER="rec -q -r 16000 -b 16 -c 1 -e signed-integer --endian little -t wav -"
PYTHON="python3.5"
TEE="tee rec.wav"

if [[ ! -f "$PWD/run.sh" ]]; then echo "This script should be run from the bin/ folder or with \"roslaunch voice_recognition main.launch\""; exit 1; fi

if [[ ! -z "$1" ]]; then
    RECORDER="eval pv -q -L 16000 < $1"
    TEE="cat"
else
    echo "Double check that the script is using the correct audio input device" \
         "by using the built in PulseAudio settings panel or pavucontrol"
fi

cat ../data/models/$MODEL/desc.txt

$RECORDER | $TEE | \
    $PYTHON -u ../src/splitter/splitter.py -t 5.0,1.0 | \
    $PYTHON -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    $PYTHON -u ../src/filter/confidence_filter.py | \
    cat # $PYTHON -u ../src/filter/chain_filter.py | \
    # $PYTHON -u ../src/publisher/publisher.py
