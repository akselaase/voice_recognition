import os
import logging
import sys
import math
import itertools
from enum import Enum
from window import Window


def find_loudest_interval(data, count, maxlength):
    scanner = Window(maxlength, data)
    maxind = 0
    maxval = scanner.sum()
    while scanner.end() < count:
        scanner.step()
        if scanner.sum() > maxval:
            maxind, maxval = scanner.start(), scanner.sum()
    return maxind


def process_stream(stream, samplerate, **kwargs):
    data_cb = kwargs.get('data_cb', None)
    auto_adjust = kwargs.get('auto_adjust_duration', 0)
    # The minimum value of the window to be considered a word
    wordthreshold = kwargs.get('t_loudness', 0.15)
    # The maximum value of the window to be considered silence
    silencethreshold = kwargs.get('t_silence', 0.03)

    minwordsamples = int(0.05 * samplerate)
    minsilencesamples = int(0.1 * samplerate)
    # Clip words to 0.75 seconds with maximized volume
    maxclipsamples = int(0.75 * samplerate)
    pre_window_samples = maxclipsamples // 4
    post_window_samples = maxclipsamples // 4
    minclipsamples = pre_window_samples + minwordsamples + post_window_samples

    if auto_adjust > 0:
        average = 0
        logging.info(
            'Running splitter auto adjustment for {} seconds.'.format(auto_adjust))
        adjust_samples = math.ceil(auto_adjust * samplerate)
        for sample in itertools.islice(stream, adjust_samples):
            average += sample ** 2
        average = math.sqrt(average / adjust_samples) / 32768.0
        wordthreshold = average * 10
        silencethreshold = average * 2
        logging.info('Average amplitude after {} seconds was {:.3f}'.format(
            auto_adjust, average))

    logging.info('Word threshold is {:.3%} and silence threshold is {:.3%}.'.format(
        wordthreshold, silencethreshold))

    wordthreshold *= 32767
    silencethreshold *= 32767

    windowsamples = minwordsamples
    wordthreshold = windowsamples * wordthreshold ** 2
    silencethreshold = windowsamples * silencethreshold ** 2
    samples = []
    window = Window(windowsamples, samples)

    def step():
        try:
            sample = next(stream)
        except StopIteration:
            return False
        sample = min(max(sample, -32768), 32767)
        samples.append(sample)
        window.step()
        return True

    def maximize(min_steps=0):
        while not window.is_maximized():
            if not step():
                return False
        while min_steps > 0:
            if not step():
                return False
            min_steps -= 1
        return True

    while True:
        # Scan until we find a long enough whole word
        logging.debug('Looking for a word.')

        word_start = window.start()
        while window.end() - word_start < minwordsamples or window.sum() < wordthreshold:
            # Ensure the window is maximized, stepping
            # forward at least once.
            if not maximize(1):
                return
            if window.sum() < wordthreshold:
                # If the window was too silent, discard data up to
                # the start of the window and start looking again.
                # We don't discard up to the end, because that could
                # include the start of a word.
                # Still, ensure that we have at least pre_window_samples left.

                discard_index = max(0, window.start() - pre_window_samples)
                del samples[:discard_index]
                window.movetoend()
                word_start = window.start()

        # We got a whole word now stored in our samples array,
        # maybe including some silence at the start.

        # Scan until we find the end of the word. That is; extend the number of
        # samples included in the word beyond just minwordsamples, but all the
        # way until a silent window is found. Note: if this check is in
        # place, it may take a long time before the word is saved to a file if
        # there is a lot of high-amplitude noise in the signal. If this check is
        # skipped, high-amplitude noise might be interpreted as the start of a
        # word, leading to the word being cut off (at minwordsamples).
        logging.debug('Looking for short silence.')
        while window.sum() > silencethreshold:
            if not step():
                return

        while window.end() - word_start < minclipsamples:
            if not step():
                return

        # Find the loudest interval in our array and save it
        index = find_loudest_interval(
            samples, window.end(), maxclipsamples)
        data_cb(samplerate, samples[index:index+maxclipsamples])

        # We've saved the word, now we just have to find a long
        # enough silence to separate this and the next word.

        logging.debug('Looking for long silence.')

        samples.clear()
        window.movetostart()
        silence_start = 0
        # Scan until we find a long enough continuous silence
        # While the length is still too short, ensure the window
        # is maximized while performing at least one step forward,
        # and check if the window is still silent enough. If not -
        # reset and start again.
        while window.end() - silence_start < minsilencesamples:
            if not maximize(1):
                return
            if window.sum() > silencethreshold:
                silence_start = window.start()
        logging.debug('Found silence!')
