#!/bin/bash
export TF_CPP_MIN_LOG_LEVEL=3
export ROS_IP=$(ip route get 1 | awk '{print $7;exit}')
export ROS_MASTER_URI=http://localhost:11311/
MODEL=arse_220k
RECORDER="rec -q --buffer 1024 -r 16000 -b 16 -c 1 -e signed-integer --endian little -t wav -"

# cd "$(dirname "$(readlink -f "$0")")"

if [ ! -f "$PWD/run.sh" ]; then 
    echo "This script should be run from the bin/ folder or with \"roslaunch voice_recognition main.launch\"" 1>&2
    echo "You can uncomment the above cd-command if this message is annoying." 1>&2
    exit 1
fi

if [[ -z "$VIRTUAL_ENV" && -f "../venv/bin/activate" ]]; then
	. ../venv/bin/activate
fi

echo "Double check that the script is using the correct audio input device" \
     "by using the built in PulseAudio settings panel or pavucontrol" 1>&2

cat ../data/models/$MODEL/desc.txt 1>&2
echo --- 1>&2

$RECORDER | \
	python3 -u ../src/splitter/splitter.py -t $(cat thresholds.txt) | \
    python3 -u ../src/labeler/label_wav.py \
        --graph=../data/models/$MODEL/graph.pb \
        --labels=../data/models/$MODEL/conv_labels.txt \
        --num_outputs=2 | \
    python3 -u ../src/filter/confidence_filter.py | \
    python3 -u ../src/filter/chain_filter.py | \
    python3 -u ../src/publisher/publisher.py "$@"
