#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
MODEL=arse

cat ../data/models/$MODEL/desc.txt
arecord -t wav -r 16000 -c 1 -f S16_LE -d 0 -N -- - | \
    python -u ../src/splitter/splitter.py -t 0.5,0.05 | \
    python -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    python -u ../src/filter/filter.py # | \
    #python -u ../src/publisher/publisher.py
