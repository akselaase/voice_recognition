#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
export ROS_IP=192.168.0.105
export ROS_MASTER_URI=http://192.168.0.101:11311/
export AUDIODEVICE=hw:CARD=UR22,DEV=0
MODEL=arse
#RECORDER="arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- -"
RECORDER="rec -q -r 16000 -b 16 -c 1 -e signed-integer --endian little -t wav -"
PYTHON="python3.5"
TEE="tee rec.wav"

cd /home/simengangstad/Ascend/catkin_ws/src/voice_recognition/bin
# cd $(dirname $(locate voice_recognition/bin/run.sh))

if [[ ! -z "$1" ]]; then
    RECORDER="cat $1"
    TEE="cat"
fi

cat ../data/models/$MODEL/desc.txt

$RECORDER | $TEE | \
    $PYTHON -u ../src/splitter/splitter.py -t 5.0,1.0 | \
    $PYTHON -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    $PYTHON -u ../src/filter/confidence_filter.py | \
    $PYTHON -u ../src/filter/chain_filter.py | \
    $PYTHON -u ../src/publisher/publisher.py
