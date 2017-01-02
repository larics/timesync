#!/usr/bin/env python
import rosbag
import rospy
import sys
import os

from sensor_msgs.msg import Image

if len(sys.argv) < 3:
  print "Usage: {} <bag_name> <timeshift>".format(os.path.basename(sys.argv[0]))
  exit()

bagName = sys.argv[1]
timeShift = rospy.Duration(float(sys.argv[2]))

try:
  input_bag = rosbag.Bag(bagName)
  bag_loaded = True
except Exception:
  print "Could not open bag {}".format(bagName)
  bag_loaded = False

if bag_loaded:
  name, ext = os.path.splitext(os.path.basename(bagName))
  outBagName = "{name}_timeshifted{ext}".format(name=name, ext=ext)
  timeshifted_bag = rosbag.Bag(outBagName, 'w')

  for topic, msg, t in input_bag.read_messages(topics=['/camera/image_raw', '/imu']):
    if topic == '/camera/image_raw' or topic == '/camera/image_mono':
      msg.header.stamp += timeShift
      timeshifted_bag.write(topic, msg, t)
    if topic == '/imu':
      timeshifted_bag.write(topic, msg, t)

  timeshifted_bag.close()
  input_bag.close()
