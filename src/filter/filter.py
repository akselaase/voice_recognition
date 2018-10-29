import sys
import signal
import logging
logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:   filter.py: %(message)s')

threshold = 0.75
runnerup_diff = 0.50

signal.signal(signal.SIGINT, lambda _, __: sys.exit())


def handle_labeling(confidences):
    confidences.sort(key=lambda t: t[1], reverse=True)
    first = confidences[0]
    if first[1] < threshold:
        logging.info("'{}' ({:.2%}) didn't pass threshold.".format(*first))
        return
    elif len(confidences) > 1:
        second = confidences[1]
        if first[1] - second[1] < runnerup_diff:
            logging.info("'{}' ({:.2%}) didn't pass filter, '{}' ({:.2%}) was too close.".format(
                *first, *second
            ))
            return

    logging.debug('Printed {} to stdout'.format(word))
    print(first[0])


confidences = []
for line in sys.stdin:
    line = line.rstrip()
    if len(line) == 0:
        continue
    elif line[0] == '-':
        handle_labeling(confidences)
        confidences.clear()
    else:
        parts = line.split()
        if len(parts) != 2:
            logging.warn('Malformed input "{}"'.format(line))
            continue
        word = parts[0]
        confidence = float(parts[1])
        confidences.append((word, confidence))
