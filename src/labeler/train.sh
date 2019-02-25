#!/bin/bash
echo Training model to recognize "$(cat wanted_words.txt)"
python ./tf/train.py \
    --wanted_words="$(cat ./wanted_words.txt)" \
    --max_samples_per_word=-1 \
    --testing_percentage=10 \
    --validation_percentage=10 \
    --unknown_percentage=15 \
    --silence_percentage=15 \
    --batch_size=16 \
    --data_url= \
    --data_dir=../../data/samples/arse_augmented \
    --summaries_dir=training/retrain_logs \
    --train_dir=training/speech_commands_train \
    --start_checkpoint=$1 \
    --how_many_training_steps=200000,20000 \
    --save_step_interval=4000 \
    --eval_step_interval=4000
