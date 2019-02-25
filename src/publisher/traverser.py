import os
import sys
from time import sleep
from tree import Tree, fromList
import actions
import logging
logging.basicConfig(stream=sys.stderr, level=logging.WARN,
                    format='[%(levelname)s]:traverser.py: %(message)s')

chain = ['root', None,
         ('search', actions.map_world),
         ('objectives', actions.move_to_objectives),
         ('heal', actions.heal_pilot),
         ('distract', actions.distract_enemy_drones),
         ['other', None,
          ('freeze', actions.freeze),
          ('flip', actions.do_a_flip),
          ('explode', actions.self_destruct),
          ['power', None,
           ('start', actions.power_on),
           ('stop', actions.power_off)
           ]
          ]
         ]

binary = ['root', None,
          ['movement', None,
           ['high level movement', None,
            ['takeoff & landing', None, 
             ('start', None),
             ('stop', None)],
           ['in flight', None, 
            ('search', actions.map_world),
            ('objectives', actions.move_to_objectives)
            ],
           ],
           ['low level movement', None,
            ('do a flip', actions.do_a_flip),
            ('freeze', actions.freeze)
            ]
           ],
          ['protection', None,
           ('heal', actions.heal_pilot),
           ('distract', actions.distract_enemy_drones)
           ]
          ]

root = fromList(binary)
current = root

#print("traverser.py: in tree '{}'".format(current.get_name()))
current.print_tree()

def clear():
    os.system('clear')

for line in sys.stdin:
    cmd = line.strip()
    print()
    #print('Received command', cmd)
    logging.debug('Received command {}'.format(cmd))

    if cmd == 'root':
        current = root
    elif cmd == 'up':
        current = current.parent
    elif cmd == 'left':
        current = current.children[0]
    elif cmd == 'right':
        current = current.children[-1]
    else:
        child = current.get_child_by_name(cmd, recursive=True)
        if not child:
            print("Unknown command '{}'".format(cmd))
        else:
            current = child

    if not current.has_children():
        print('You selected', current.get_name())
        action = current.get_value()
        if action:
            action()
        input()
        current = root

    clear()
    current.print_tree()