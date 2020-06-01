import rclpy
from rclpy.node import Node
import click
import sys
import os
import threading
from librosshow.getch import Getch
import librosshow.termgraphics as termgraphics
import time

VIEWER_MAPPING = {
  "nav_msgs/Odometry": ("librosshow.viewers.nav_msgs.OdometryViewer", "OdometryViewer", {}),
  "nav_msgs/OccupancyGrid": ("librosshow.viewers.nav_msgs.OccupancyGridViewer", "OccupancyGridViewer", {}),
  "nav_msgs/Path": ("librosshow.viewers.nav_msgs.PathViewer", "PathViewer", {}),
  "std_msgs/Bool": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Float32": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Float64": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Int8": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Int16": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Int32": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/Int64": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/UInt8": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/UInt16": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/UInt32": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "std_msgs/UInt64": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
  "sensor_msgs/CompressedImage": ("librosshow.viewers.sensor_msgs.CompressedImageViewer", "CompressedImageViewer", {}),
  "sensor_msgs/FluidPressure": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "fluid_pressure"}),
  "sensor_msgs/RelativeHumidity": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "relative_humidity"}),
  "sensor_msgs/Illuminance": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "illuminance"}),
  "sensor_msgs/msg/Image": ("librosshow.viewers.sensor_msgs.ImageViewer", "ImageViewer", {}),
  "sensor_msgs/Imu": ("librosshow.viewers.sensor_msgs.ImuViewer", "ImuViewer", {}),
  "sensor_msgs/LaserScan": ("librosshow.viewers.sensor_msgs.LaserScanViewer", "LaserScanViewer", {}),
  "sensor_msgs/NavSatFix": ("librosshow.viewers.sensor_msgs.NavSatFixViewer", "NavSatFixViewer", {}),
  "sensor_msgs/PointCloud2": ("librosshow.viewers.sensor_msgs.PointCloud2Viewer", "PointCloud2Viewer", {}),
  "sensor_msgs/Range": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "range"}),
  "sensor_msgs/Temperature": ("librosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "temperature"}),
}

class ROS2Show(Node):

    def __init__(self, topic, ascii_enable=None, color_mode=None):
        super().__init__('ros2show') 
        if ascii_enable is None:
            self.ascii_enable = True
        else:
             self.ascii_enable = ascii_enable 
        if color_mode is None:
            self.color_mode = None
        else:
             self.color_mode = color_mode 
   
        time.sleep(3)
        topic_types = dict(self.get_topic_names_and_types())    
        topic_type=', '.join(topic_types[topic])
        if topic not in topic_types:
            print("Topic {0} does not appear to be published yet.".format(topic))
            sys.exit(0)
        if topic_type not in VIEWER_MAPPING:
            print("Unsupported message type.")
            exit()
            
        # Create the canvas and viewer accordingly
        self.canvas = termgraphics.TermGraphics( \
            mode = (termgraphics.MODE_EASCII if ascii_enable else termgraphics.MODE_UNICODE),
            color_support = color_mode)

        module_name, class_name, viewer_kwargs = VIEWER_MAPPING[topic_type]
        viewer_class = getattr(__import__(module_name, fromlist=(class_name)), class_name)
        self.viewer = viewer_class(self.canvas, title = topic, **viewer_kwargs)

        message_package, message_name = topic_type.split("/msg/", 2)
        message_class = getattr(__import__(message_package + ".msg", fromlist=(message_name)), message_name)

        self.sub = self.create_subscription(message_class, topic, self.viewer.update, 10)
        
        self.getch = Getch()
        self._thread = threading.Thread(target=self._run, args=(), daemon=False)
        self._thread.start()
        
    def _run(self):
        while True:
            c = self.getch()
            print("c",c)
            if c == '\x03': # Ctrl+C
                print("ctrl+C")
                break

            if "keypress" not in dir(self.viewer):
                continue

            if c == '\x1B': # ANSI escape
                c = self.getch()
                if c == '\x5B':
                    c = self.getch()
                    if c == '\x41':
                        self.viewer.keypress("up")
                    if c == '\x42':
                        self.viewer.keypress("down")
                    if c == '\x43':
                        self.viewer.keypress("left")
                    if c == '\x44':
                        self.viewer.keypress("right")
            else:
                self.viewer.keypress(c)    
        print("loop")    
                
@click.command()
@click.option('-a', '--ascii', is_flag=True, help="Enable ASCII only, no unicode")
@click.option('-c', '--color-mode', type=click.Choice(['mono', 'c4', 'c24']), 
              help=(
                  'mono - For monochrome'
                  'c4   - For 4-bit color'
                  'c24  - For 24-bit color'
                  )
              )
@click.argument('topic', required=True)
def main(ascii, color_mode, topic, args=None):
    if color_mode == "mono":
        color_support = termgraphics.COLOR_SUPPORT_1
    elif color_mode == "c4":
        color_support = termgraphics.COLOR_SUPPORT_16
    elif color_mode == "c24":
        color_support = termgraphics.COLOR_SUPPORT_24BIT
    else:
        color_support = None # TermGraphics class will autodetect
 
 
    rclpy.init(args=args)
    node = ROS2Show(topic, ascii, color_support)
    
    # Drawing loop
    frame_rate = 15.
    frame_duration = 1. / frame_rate
    try:
        while rclpy.ok():
            start_time = time.time()
            #node.viewer.draw()
            stop_time = time.time()
            draw_time = stop_time - start_time
            delay_time = max(0, frame_duration - draw_time)
            rclpy.spin_once(node, timeout_sec=delay_time)
    except KeyboardInterrupt:
        pass
    finally:
        print("cleaning")
        node.getch.reset()
        node.destroy_node()
        if rclpy.ok():
            print("shutdown")
            rclpy.try_shutdown()


if __name__ == '__main__':            
    main()