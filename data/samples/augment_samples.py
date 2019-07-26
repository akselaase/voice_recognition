import os
import sys
import shutil
import random
import subprocess
import itertools
import tempfile

clean_output_dirs = False
io_dirs = {
        'arse_cleaned': 'samples_augmented', 
        'aiy_speech_dataset': 'samples_augmented'
        }
target_count_per_word = 5000


def randfloat(min=-1.0, max=1.0):
    return min + (max - min) * random.random()


def shuffle_indefinitely(effects):
    queue = list(effects)
    while True:
        random.shuffle(queue)
        yield from queue


def list_replace(iterable, mapping):
    res = []
    for item in iterable:
        replaced = mapping.get(item, item)
        res.append(replaced)
    return res


def pitch(min, max):
    while True:
        pitch = randfloat(min, max)
        yield ['pitch', str(pitch)]


def tempo(min, max):
    while True:
        tempo = randfloat(min, max)
        yield ['tempo', str(tempo)]


def distort(min, max):
    while True:
        gain = randfloat(min, max)
        yield ['norm', 'gain', str(gain)]


def echo(min_ms, max_ms, min_decay, max_decay):
    while True:
        delay = random.randint(min_ms, max_ms)
        decay = randfloat(min_decay, max_decay)
        yield ['echo', '1', '1', str(delay), str(decay)]


def lowpass(min_freq, max_freq):
    while True:
        freq = random.randint(min_freq, max_freq)
        yield ['lowpass', str(freq)]


def highpass(min_freq, max_freq):
    while True:
        freq = random.randint(min_freq, max_freq)
        yield ['highpass', str(freq)]


def bandpass(low_min, low_max, band_min, band_max):
    while True:
        freq_min = random.randint(low_min, low_max)
        freq_max = freq_min + random.randint(band_min, band_max)
        yield ['sinc', str(freq_min) + '-' + str(freq_max)]


def bandreject(low_min, low_max, mul_min, mul_max):
    while True:
        freq_min = random.randint(low_min, low_max)
        freq_max = int(freq_min * randfloat(mul_min, mul_max))
        yield ['sinc', str(freq_max) + '-' + str(freq_min)]


# def bgmix(vol_min, vol_max):
#     while True:
#         vol = randfloat(vol_min, vol_max)
#         yield ['trim', '0', '1']


pass1_effects = [
    pitch(-200, 350),
    tempo(1.15, 1.3),
    distort(2.0, 6.0),
    echo(1, 20, 0.6, 0.9),
    lowpass(2000, 3000),
    highpass(600, 1200),
    bandpass(100, 750, 2000, 5000),
    bandreject(300, 1000, 1.5, 2.0),
]
# pass2_effects = [
#     bgmix(0.1, 0.5)
# ]

# (group_of_effects, target_file_count)
effect_passes = [
    (pass1_effects, target_count_per_word),
    # (pass2_effects, target_count_per_word)
]


def get_output_file(input_file, file_output_dir, effect_name):
    fname, ext = os.path.splitext(os.path.basename(input_file))
    output_file_base = os.path.join(file_output_dir, fname + '_' + effect_name)
    i = 0
    while True:
        output_file = output_file_base + '_' + str(i) + ext
        if not os.path.exists(output_file):
            return output_file
        i += 1


def sox(input_file, output_file, effect_args):
    if effect_args[0] == 'sox':
        substitutions = {'$INPUT': input_file, '$OUTPUT': output_file}
        args = list_replace(effect_args, substitutions)
    else:
        args = ['sox', input_file, output_file, *effect_args]
    try:
        subprocess.run(args=args, check=True, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as ex:
        print('{} on {} failed with code {}, effect: {}'.format(
            effect_args[0], input_file, ex.returncode, effect_args))


def augment_file(input_file, file_output_dir, effects, multiplier):
    for effect in itertools.islice(shuffle_indefinitely(effects), multiplier):
        output_file = get_output_file(
            input_file, file_output_dir, effect.__name__)
        sox(input_file, output_file, next(effect))


def perform_pass(pass_input_dir, pass_output_dir, effect_pass):
    os.makedirs(pass_output_dir, exist_ok=True)
    effects, target_file_count = effect_pass
    all_files = list(filter(lambda entry: entry.is_file() and entry.name.endswith('.wav'),
                            os.scandir(pass_input_dir)))
    multiplier = min(int(target_file_count / len(all_files) + 0.5), 500)
    print('Multiplier: {}, Num Files: {}'.format(multiplier, min(multiplier * len(all_files), target_file_count)))
    for input_file in all_files:
        file_multiplier = min(target_file_count, multiplier)
        augment_file(input_file.path, pass_output_dir,
                     effects, file_multiplier)
        target_file_count -= file_multiplier


def perform_all_passes(input_dir, output_dir):
    pass_input_dir = input_dir
    tmp_input_dir, tmp_output_dir = None, None
    for index, effect_pass in enumerate(effect_passes):
        if index < len(effect_passes) - 1:
            tmp_output_dir = tempfile.TemporaryDirectory()
            pass_output_dir = tmp_output_dir.name
        else:
            tmp_output_dir = None
            pass_output_dir = output_dir

        print('Pass', index, 'on', pass_input_dir, '-->', pass_output_dir)
        perform_pass(pass_input_dir, pass_output_dir, effect_pass)

        if tmp_input_dir:
            tmp_input_dir.cleanup()
        tmp_input_dir = tmp_output_dir
        pass_input_dir = pass_output_dir


def augment_all(words=[]):
    for input_dir, output_dir in io_dirs.items():
        for word in filter(lambda d: d.is_dir(), os.scandir(input_dir)):
            if not words or word.name in words:
                word_output_dir = os.path.join(output_dir, word.name)
                perform_all_passes(word.path, word_output_dir)


def test_effect(input_file, effect):
    while True:
        try:
            output_file = '/tmp/out.wav'
            effect_args = next(effect)
            sox(input_file, output_file, effect_args)
            print(effect_args)
            subprocess.run(args=['aplay', '/tmp/out.wav'],
                           stderr=subprocess.DEVNULL)
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    words = sys.argv[1:]
    if not words:
        ans = input('Confirm intention to augment all data (y/N):')
        if ans != 'y':
            sys.exit()
    augment_all(words)
