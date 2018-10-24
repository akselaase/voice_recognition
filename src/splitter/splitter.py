import os
import sys
import wave
import struct
import signal
import logging
import argparse
import tempfile
from worker import process_stream

logging.basicConfig(stream=sys.stderr, level=logging.INFO,
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

    stream = readwav(sys.stdin.buffer)
    rate = next(stream)
    process_stream(stream, rate, auto_adjust=5, data_cb=save_loud_area)

    logging.info('Exiting splitter.py')
    if file_count == 0:
        sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("splitter.py")
    parser.add_argument("-o", "--output_dir",
                        default=tempfile.gettempdir() + '/splitter.py')

    FLAGS = parser.parse_args()
    main()
