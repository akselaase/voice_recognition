#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
export ROS_MASTER_URI=http://localhost:11311/
export AUDIODEVICE=plughw:CARD=UR22,DEV=0
MODEL=arse
#RECORDER="arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- -"
RECORDER="rec -q -r 16000 -b 16 -c 1 -e signed-integer --endian little -t wav -"
PYTHON=python3.5

cat ../data/models/$MODEL/desc.txt

$RECORDER | \
    $PYTHON -u ../src/splitter/splitter.py -t 5.0,1.0 | \
    $PYTHON -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    $PYTHON -u ../src/filter/confidence_filter.py  | \
    $PYTHON -u ../src/publisher/publisher.py
