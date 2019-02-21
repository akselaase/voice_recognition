import sys
import logging

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG,
                    format='[%(levelname)s]:chain_filter.py: %(message)s')


def matches_chain(candidate):
    l = len(chain)
    return chain == candidate[:l]


def equals_chain(candidate):
    return chain == candidate


known_words = ['alpha', 'bravo', 'charlie', 'delta', 'echo',
               'foxtrot', 'golf', 'hotel', 'india', 'juliett']
#input_chains = list(
#    map(list, zip(known_words, known_words[1:] + known_words[0:1])))
input_chains = list(map(lambda s: [s], known_words))
output_commands = list(map(str, range(10)))

assert len(input_chains) == len(output_commands)

chain_length = max(map(len, input_chains))

chain = []
matches = input_chains
try:
    for line in sys.stdin:
        word = line.strip()
        if word == '_silence_' or word == '_unknown_':
            logging.debug('Ignoring "%s"', word)
            continue
        chain.append(word)

        matches = list(filter(matches_chain, input_chains))
        exact_match = next(filter(equals_chain, input_chains), None)
        if exact_match:
            match_index = input_chains.index(exact_match)
            logging.info('Exact match: %i, %s', match_index, str(exact_match))
            print(output_commands[match_index])
            chain.clear()
        elif not matches:  # or len(chain) == chain_length:
            logging.info('No matches, clearing.')
            chain = []
        logging.debug('Current chain: ' + ' '.join(chain))
except KeyboardInterrupt:
    pass
