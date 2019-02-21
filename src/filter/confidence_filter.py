import sys
import signal
import logging
logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:filter.py: %(message)s')

threshold = 0.6
runnerup_diff = 0.4

signal.signal(signal.SIGINT, lambda _, __: sys.exit())


def handle_labeling(confidences):
    # Sort word/confidence pairs by confidence descending
    # and check if the first one passes our thresholds
    confidences.sort(key=lambda t: t[1], reverse=True)
    first = confidences[0]
    second = confidences[1] if len(confidences) > 1 else ('', 0)
    if first[1] < threshold:
        logging.info("'{}' ({:.2%}) didn't pass threshold.".format(*first))
    elif first[1] - second[1] < runnerup_diff:
        logging.info("'{}' ({:.2%}) didn't pass filter, '{}' ({:.2%}) was too close.".format(
            *first, *second
        ))
    else:
        logging.debug('Printed {} to stdout'.format(first[0]))
        print(first[0])

# Read lines from stdin in the following format:
#   word1, 0.75
#   word2, 0.66
#   word3, 0.01
#   ---
# (repeat)
# Add each word/confidence pair to a list, and when '---'
# is encountered, process this list and start over.


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
