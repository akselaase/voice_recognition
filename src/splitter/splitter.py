import os
import sys
import wave
import struct
import signal
import logging
import argparse
import tempfile
from worker import process_stream

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG,
                    format='[%(levelname)s]: splitter.py: %(message)s')


running = True
file_count = 0

FLAGS = None


def sighandler(signal, frame):
    global running
    running = False


def readwav(source):
    with wave.open(source, 'rb') as source:
        channels = source.getnchannels()
        samplesize = source.getsampwidth()
        rate = source.getframerate()
        if channels != 1 or samplesize != 2:
            raise Exception('Unsupported format.')

        yield rate

        while running:
            data = source.readframes(1)
            if len(data) == 0:
                logging.info('Read zero frames from input file.')
                return
            sample, = struct.unpack('<h', data)
            yield sample


def save_loud_area(rate, data):
    global file_count
    outfile = os.path.join(FLAGS.output_dir, str(file_count) + '.wav')
    with wave.open(outfile, mode='wb') as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(rate)
        target.setnframes(len(data))
        for sample in data:
            conv = struct.pack('<h', sample)
            target.writeframesraw(conv)
    logging.debug('Written {} samples to {}'.format(len(data), outfile))
    print(outfile, flush=True)
    file_count += 1


def main():
    logging.info('Started splitter.py')
    signal.signal(signal.SIGINT, sighandler)

    if not os.path.exists(FLAGS.output_dir):
        os.makedirs(FLAGS.output_dir)

    auto_adjust = True
    loudness = 30
    silence = 5
    if FLAGS.thresholds is not None:
        auto_adjust = False
        parts = FLAGS.thresholds.split(',')
        if len(parts) == 0:
            logging.warn(
                "Couldn't parse thresholds, resorting to auto adjustment.")
            auto_adjust = True
        try:
            loudness = float(parts[0])
            if len(parts) > 1:
                silence = float(parts[1])
        except ValueError:
            logging.warn(
                "Couldn't parse thresholds, resorting to auto adjustment.")
            auto_adjust = True

    files = []
    if FLAGS.input == '-':
        files = [sys.stdin.buffer]
    elif os.path.exists(FLAGS.input):
        if os.path.isdir(FLAGS.input):
            for f in os.listdir(FLAGS.input):
                files.append(os.path.join(FLAGS.input, f))
        elif os.path.isfile(FLAGS.input):
            files.append(FLAGS.input)
    else:
        logging.error("Didn't find file {}".format(FLAGS.input))
        return

    for f in files:
        close = False
        if type(f) is str:
            logging.info('Running splitter.py on "{}"'.format(f))
            f = open(f, 'rb')
            close = True

        stream = readwav(f)
        rate = next(stream)
        if auto_adjust:
            process_stream(stream, rate, data_cb=save_loud_area,
                           auto_adjust=FLAGS.auto_adjust)
        else:
            process_stream(stream, rate, data_cb=save_loud_area, t_loudness=loudness /
                           100, t_silence=silence / 100)

        if close:
            f.close()

    logging.info('Exiting splitter.py')
    if file_count == 0:
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("splitter.py")
    parser.add_argument("-o", "--output_dir",
                        default=tempfile.gettempdir() + '/splitter.py')
    parser.add_argument("-t", "--thresholds",
                        default=None)
    parser.add_argument("-a", "--auto-adjust", type=int,
                        default=5)
    parser.add_argument("-i", "--input",
                        default="-")
    FLAGS = parser.parse_args()
    main()
