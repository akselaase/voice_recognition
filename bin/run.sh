#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- - | \
    python3.6 -u ../src/splitter/splitter.py | \
    python3.6 -u ../src/labeler/label_wav.py \
        --graph=../data/models/1/graph.pb \
        --labels=../data/models/1/conv_labels.txt \
        --num_outputs=1 | \
    python3.6 -u ../src/filter/filter.py | \
    python3.6 -u ../src/publisher/publish.py
