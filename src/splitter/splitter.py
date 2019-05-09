import os
import sys
import wave
import struct
import signal
import pickle
import logging
import argparse
import tempfile
from worker import process_stream, sums

logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:splitter.py: %(message)s')


running = True
file_count = 0

FLAGS = None


def sighandler(signal, frame):
    global running
    running = False


def readwav(source):
    with wave.open(source, 'rb') as wf:
        channels = wf.getnchannels()
        samplesize = wf.getsampwidth()
        rate = wf.getframerate()
        if channels != 1 or samplesize != 2:
            raise Exception('Unsupported format.')
        def _stream_reader():
            buff_size = 64
            while running:
                data = wf.readframes(buff_size)
                num_samples = len(data) // 2
                if num_samples == 0:
                    logging.info('Read zero frames from input file.')
                    return
                samples = struct.unpack('<{}h'.format(num_samples), data)
                yield from samples
        return rate, _stream_reader()


def save_loud_area(rate, data):
    global file_count
    outfile = os.path.join(FLAGS.output_dir, str(file_count) + '.wav')
    with wave.open(outfile, mode='wb') as target:
        num_samples = len(data)
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(rate)
        target.setnframes(num_samples)
        conv = struct.pack('<{}h'.format(num_samples), *data)
        target.writeframesraw(conv)
    logging.debug('Wrote {} samples to {}'.format(num_samples, outfile))
    print(outfile, flush=True)
    file_count += 1


def main():
    logging.debug('Started splitter.py')
    signal.signal(signal.SIGINT, sighandler)

    if not os.path.exists(FLAGS.output_dir):
        os.makedirs(FLAGS.output_dir)

    auto_adjust = True
    loudness = 30
    silence = 5
    if FLAGS.thresholds is not None:
        parts = FLAGS.thresholds.split(',')
        if len(parts) == 0:
            logging.warn(
                "Couldn't parse thresholds, resorting to auto adjustment.")
        else:
            try:
                loudness = float(parts[0])
                if len(parts) > 1:
                    silence = float(parts[1])
                auto_adjust = False
            except ValueError:
                logging.warn(
                    "Couldn't parse thresholds, resorting to auto adjustment.")

    rate, stream = readwav(sys.stdin.buffer)
    if auto_adjust:
        process_stream(stream, rate, data_cb=save_loud_area,
                       auto_adjust_duration=FLAGS.auto_adjust)
    else:
        process_stream(stream, rate, data_cb=save_loud_area,
                       t_loudness=loudness / 100, t_silence=silence / 100)

    pickle.dump(sums, open('sums.bin', 'wb'))

    logging.debug('Exiting splitter.py')
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
