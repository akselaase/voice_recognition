import sys
import logging

logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:chain_filter.py: %(message)s')


def matches_chain(candidate):
    l = len(chain)
    return chain == candidate[:l]


def equals_chain(candidate):
    return chain == candidate


known_words = ['alpha', 'bravo', 'charlie', 'delta', 'echo',
               'foxtrot', 'golf', 'hotel', 'india', 'juliett']
chain_length = 1
input_chains = list()
for index in range(len(known_words)):
    input_chains.append(tuple((known_words * 2)[index:index+chain_length])) 
output_commands = {chain: chain[0] for chain, index in zip(input_chains, range(len(input_chains)))}

assert len(input_chains) == len(output_commands)

chain_length = max(map(len, input_chains))

chain = tuple()
matches = input_chains
try:
    for line in sys.stdin:
        word = line.strip()
        if word == '_silence_' or word == '_unknown_':
            logging.debug('Ignoring "%s"', word)
            continue
        chain = (*chain, word)

        matches = list(filter(matches_chain, input_chains))
        exact_match = next(filter(equals_chain, input_chains), None)
        if exact_match:
            matching_command = output_commands[exact_match]
            logging.info('Exact match: %s maps to %s', str(exact_match), matching_command)
            print(matching_command)
            chain = tuple()
        elif not matches:  # or len(chain) == chain_length:
            logging.info('No matches, clearing.')
            chain = tuple()
        if len(chain) > 0:
            logging.info('Current chain: ' + ' '.join(chain))
        else:
            logging.debug('Current chain: ' + ' '.join(chain))
except KeyboardInterrupt:
    pass
