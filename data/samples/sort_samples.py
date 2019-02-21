import os
import shutil
import random
import string

input_dir = 'arse_raw'
output_dir = 'arse_cleaned'

if not hasattr(random, 'choices'):
    def _choices(population, weights=None, *, cum_weights=None, k=1):
        return [random.choice(population) for i in range(k)]
    random.choices = _choices


def find_available_name(dir):
    name = os.path.join(dir, ''.join(
        random.choices(string.ascii_lowercase, k=6)) + '.wav')
    while os.path.isfile(name):
        name = os.path.join(dir, ''.join(
            random.choices(string.ascii_lowercase, k=6)) + '.wav')
    return name


def fix_word(word):
    word = word.lower()
    replacements = {'alfa': 'alpha',
                    'juliet': 'juliett', 'juliettt': 'juliett'}
    for key, val in replacements.items():
        word = word.replace(key, val)
    return word


def copy_samples(word):
    source_path = os.path.join(word.path, 'words')
    target_path = os.path.join(output_dir, fix_word(word.name))
    if not os.path.isdir(source_path):
        return
    created = False
    for sample in os.scandir(source_path):
        if sample.is_file(follow_symlinks=False):
            if not created:
                os.makedirs(target_path, exist_ok=True)
                created = True
            target_file = find_available_name(target_path)
            print('Copy {} to {}'.format(sample.path, target_file))
            shutil.copy(sample.path, target_file)


def main():
    for person in os.scandir(input_dir):
        if person.is_dir():
            for word in os.scandir(person.path):
                if word.is_dir():
                    copy_samples(word)


if __name__ == '__main__':
    main()
