import sys
import argparse
import logging

logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                    format='[%(levelname)s]:publisher.py: %(message)s')

stdout = False
try:
    if '--stdout' in sys.argv:
        logging.info('Using stdout as publisher target.')
        stdout = True
        import non_existent_random_package_name_to_trigger_exception

    import rospy
    import std_msgs
    rospy.init_node('voice_recognition')
    pub = rospy.Publisher('/ai/voice/command',
                          std_msgs.msg.String, queue_size=10, latch=True, tcp_nodelay=True)

    def publish(line):
        print(line)
        pub.publish(line)
    exit_condition = rospy.is_shutdown
except ImportError:
    if not stdout:
        logging.warning(
            'Failed to import rospy or std_msgs, will publish to stdout instead.')
    publish = print
    def exit_condition():
        return False

while not exit_condition():
    try:
        line = input()
    except (EOFError, KeyboardInterrupt):
        break
    publish(line)
