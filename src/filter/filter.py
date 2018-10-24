import sys
import signal
import logging
logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:   filter.py: %(message)s')

threshold = 0.75

signal.signal(signal.SIGINT, lambda _, __: sys.exit())

for line in sys.stdin:
    line = line.rstrip()
    parts = line.split()
    if len(parts) != 2:
        logging.warn('Malformed input "{}"'.format(line))
        continue
    word = parts[0]
    confidence = float(parts[1])
    if confidence > threshold:
        logging.debug('Printed {} to stdout'.format(word))
        print(word)
    else:
        logging.info("'{}' didn't pass filter.".format(line))
