#!/bin/bash
echo Training model to recognize "$(cat wanted_words.txt)"
python ./tf/train.py \
    --wanted_words="$(cat ./wanted_words.txt)" \
    --max_samples_per_word=-1 \
    --testing_percentage=10 \
    --validation_percentage=10 \
    --unknown_percentage=25 \
    --data_url= \
    --data_dir=../../data/samples/arse \
    --summaries_dir=training/retrain_logs \
    --train_dir=training/speech_commands_train \
    --start_checkpoint=$1
