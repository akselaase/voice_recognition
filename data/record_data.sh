date=$(date '+%Y-%m-%d_%H:%M:%S')
name=$1
word=$2
if [ -z "$1" ]; then
    echo "What is your name?"
    read name
fi
if [ -z "$2" ]; then
    echo "Which word are you recording?"
    read word
fi

shopt -s
trap ctrl_c INT

shouldexit="0"
function ctrl_c() {
    if [ "$shouldexit" == "1" ]; then
        exit
    else
        shouldexit="1"
    fi
}

outdir="custom/$name/$date $word"
echo "Recording data to $outdir"
mkdir -p "$outdir"

arecord -t wav -r 48000 -c 1 -f S16_LE -- - | \
    tee "$outdir"/raw.wav | \
    python3.6 -u ../splitter/splitter.py -o "$outdir"/split

if [ $? -ne 0 ]; then
    echo "No files saved."
    exit
fi

echo "Saved all files to $outdir"
echo "Listen through and filter the files now (Yes/no)?"
while :; do
    read input
    if [ "$input" == "n" ] || [ "$input" == "no" ]; then
        echo "Exiting script."
        exit
    elif [ "$input" == "y" ] || [ "$input" == "yes" ] || [ "$input" == "" ]; then
        break
    else
        echo "Didn't understand your input. Type lowercase 'yes' or 'no'."
    fi
done

echo "Continuing to filtering stage."

for f in "$outdir"/split/*.wav; do
    while :; do
        aplay "$f"
        echo "Keep file (Yes/no/replay)?"
        read input
        if [ "$input" == "r" ] || [ "$input" == "replay" ]; then
            continue
        elif [ "$input" == "y" ] || [ "$input" == "yes" ] || [ "$input" == "" ]; then
            echo "Keeping file"
            break
        elif [ "$input" == "n" ] || [ "$input" == "no" ]; then
            echo "Running rm \"$f\""
            rm "$f"
            break
        else
            echo "Didn't understand your input. Type lowercase 'yes', 'no' or 'replay'."
        fi
    done
done