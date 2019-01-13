#!/bin/bash
echo Training model to recognize "$(cat wanted_words.txt)"
python ./tf/train.py \
    --wanted_words="$(cat ./wanted_words.txt)" \
    --max_samples_per_word=5000 \
    --testing_percentage=10 \
    --validation_percentage=10 \
    --data_dir=../../data/samples/ascend \
    --summaries_dir=/tmp/retrain_logs \
    --train_dir=/tmp/speech_commands_train \
    --start_checkpoint=$1
