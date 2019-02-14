import os
import random
import subprocess
import itertools


def randfloat(min=-1.0, max=1.0):
    return min + (max - min) * random.random()


def choose_effect(effects):
    while True:
        queue = list(effects)
        random.shuffle(queue)
        yield from queue


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


effect_groups = [
    pitch(-200, 350),
    tempo(1.15, 1.3),
    distort(2.0, 6.0),
    echo(1, 20, 0.6, 0.9),
    lowpass(2000, 3000),
    highpass(600, 1200),
    bandpass(100, 750, 2000, 5000),
    bandreject(300, 1000, 1.5, 2.0),
]
num_effects = len(effect_groups)

input_dir = 'arse_cleaned'
output_dir = 'arse_augmented'


def get_output_file(input_file, effect_name):
    base, ext = os.path.splitext(input_file)
    _, word, fname = *os.path.split(
        os.path.dirname(base)), os.path.basename(base)
    output_file_dir = os.path.join(output_dir, word)
    output_file_base = os.path.join(output_file_dir, fname + '_' + effect_name)
    os.makedirs(output_file_dir, exist_ok=True)
    i = 0
    while True:
        output_file = output_file_base + str(i) + ext
        if not os.path.exists(output_file):
            return output_file
        i += 1


def sox(input_file, output_file, effect_args):
    args = ['sox', input_file, output_file, *effect_args]
    try:
        subprocess.run(args=args, check=True, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as ex:
        print('{} on {} failed with code {}, effect: {}'.format(
            effect_args[0], input_file, ex.returncode, effect_args))


def augment_sample(input_file):
    for effect in itertools.islice(choose_effect(effect_groups), num_effects):
        output_file = get_output_file(input_file, effect.__name__)
        sox(input_file, output_file, next(effect))


def augment_all(filter_words=[]):
    for word in os.scandir(input_dir):
        if word.name in filter_words or not filter_words:
            for sample in os.scandir(word.path):
                augment_sample(sample)


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
