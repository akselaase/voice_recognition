#!/bin/bash
echo Reading wanted words from ./wanted_words.txt
python3.6 ./tf/train.py \
    --wanted_words="$(cat ./wanted_words.txt)" \
    --max_samples_per_word=500 \
    --testing_percentage=10 \
    --validation_percentage=10 \
    --data_dir=../data/aiy_speech_dataset \
    --summaries_dir=/tmp/retrain_logs \
    --train_dir=/tmp/speech_commands_train \
    --start_checkpoint=$1
