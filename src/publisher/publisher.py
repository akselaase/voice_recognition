# TODO: Do something more than just echo stdin to the ROS topic.

import sys
import argparse
import rospy
import std_msgs


def main(FLAGS, args):
    rospy.init_node('voice_recognition')
    pub = rospy.Publisher('/ai/voice/command',
                          std_msgs.msg.String, queue_size=1)

    line = ''
    while not rospy.is_shutdown():
        try:
            line = input()
        except EOFError:
            break
        pub.publish(line)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    FLAGS, unknown = parser.parse_known_args()
    main(FLAGS, unknown)
