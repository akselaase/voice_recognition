cp ./training/speech_commands_train/conv_labels.txt .
python ./tf/freeze.py \
    --start_checkpoint="$1" \
    --wanted_words=$(cat ./wanted_words.txt) \
    --output_file=./graph.pb
echo Remember to copy conv_labels.txt and graph.pb to ../data/models