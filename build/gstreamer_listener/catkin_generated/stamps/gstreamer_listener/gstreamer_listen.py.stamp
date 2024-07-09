#!/usr/bin/env python3

import gi
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

bridge = CvBridge()
current_frame = None

def image_callback(msg):
    global current_frame
    # Convert ROS Image message to OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
        # Resize if necessary to match the expected resolution
        if cv_image.shape[1] != 640 or cv_image.shape[0] != 360:
            cv_image = cv2.resize(cv_image, (640, 360))
        current_frame = cv_image
    except CvBridgeError as e:
        print(f"Error converting ROS Image message: {e}")

def cb_need_data(appsrc, length):
    global current_frame
    if current_frame is not None:
        # Convert the current frame to bytes
        data = current_frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, 1)
        appsrc.emit('push-buffer', buf)

def main():
    # Initialize ROS node
    rospy.init_node('gstreamer_image_subscriber', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/rgb_view/image', Image, image_callback)

    # Create the elements
    pipeline = Gst.Pipeline.new("pipeline")
    appsrc = Gst.ElementFactory.make("appsrc", "source")
    conv = Gst.ElementFactory.make("videoconvert", "conv")
    videosink = Gst.ElementFactory.make("autovideosink", "videosink")

    if not pipeline or not appsrc or not conv or not videosink:
        print("Not all elements could be created.")
        exit(-1)

    # Set the caps for the appsrc
    caps = Gst.Caps.from_string("video/x-raw, format=(string)RGB, width=(int)640, height=(int)360, framerate=(fraction)1/1")
    appsrc.set_property("caps", caps)

    # Build the pipeline
    pipeline.add(appsrc)
    pipeline.add(conv)
    pipeline.add(videosink)

    # Link the elements
    if not appsrc.link(conv):
        print("appsrc and conv could not be linked.")
        exit(-1)
    if not conv.link(videosink):
        print("conv and videosink could not be linked.")
        exit(-1)

    # Configure appsrc
    appsrc.set_property("stream-type", 0)
    appsrc.set_property("format", Gst.Format.TIME)
    appsrc.connect("need-data", cb_need_data)

    # Start playing
    pipeline.set_state(Gst.State.PLAYING)

    # Run the main loop
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass

    # Clean up
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    main()
