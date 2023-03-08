#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from queue import Queue
import socket
import numpy as np  # Scientific computing library for Python
import math
from rclpy.executors import MultiThreadedExecutor
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander_custom.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import websocket
import threading
import json
from nav_msgs.srv import GetMap

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class WebMonitorInterface():
    def __init__(self, subscriber, simpleCommander):
        self.subscriber = subscriber
        self.messageQ = Queue()
        self.commander = simpleCommander
        self.ws = None

    def connect(self, uri):
        # websocket.enableTrace(False)
        # self.ws = websocket.WebSocketApp(
        #     uri, on_message=self.on_message, on_close=self.on_close, on_open=self.on_open)
        # wst = threading.Thread(target=self.ws.run_forever)
        # print("ws : connecting " + uri)
        # wst.daemon = True
        # wst.start()
        self.start()

        # 스레드 객체를 생성합니다. 인자로 websocket 서버와 통신할 함수를 전달합니다.
        ws_thread = threading.Thread(target=self.ws_handler, args=(uri,))

        # 스레드를 시작합니다. start 메소드를 호출하면 스레드에서 실행할 함수가 호출됩니다.
        ws_thread.start()

    def ws_handler(self, uri):
        ws = websocket.WebSocket(enable_multithread=True)
        ws.connect(uri)
        print("ws : connecting " + uri)

        self.ws = ws
        while True:
            message = self.ws.recv()
            self.messageQ.put(message)

            # self.on_message(self, message)

    # def on_open(self, ws):
    #     self.ws = ws
    #     print("ws: Opened connection")

    def on_close(self, ws):
        print("ws: close")

    def send_message(self, type, json_message):
        # print("send message   type : " + type)
        if (self.ws != None):
            # print("send message   type : " + type)
            self.ws.send(json_message)

    def start(self):
        # print("start")
        self.subscriber.set_interface(self)
        self.commander.set_interface(self)
        # print("start done")


class SubscriberForWeb(Node):
    def __init__(self):
        super().__init__(node_name='subscriber_for_web')

        self.laser_scan_pub = self.create_subscription(
            LaserScan, "scan", self._laserScanCallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_scan_pub

        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, 10)

        self.interface = None

    def _amclPoseCallback(self, message):
        # self.info('Received amcl')
        if (self.interface != None):
            euler = self.euler_from_quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                               message.pose.pose.orientation.z, message.pose.pose.orientation.w)
            angle = euler[2]
            json_message = json.dumps(
                dict(type="amcl", positionX=message.pose.pose.position.x, positionY=message.pose.pose.position.y, positionZ=message.pose.pose.position.z,
                     angle=angle))

            # print(json_message)
            self.interface.send_message("amcl", json_message)
        return

    def _laserScanCallback(self, message):
        ranges = []
        if (self.interface != None):
            range_max = message.range_max
            for i in range(0, 360):
                if (message.ranges[i] == float("inf")):
                    ranges.append(range_max)
                else:
                    ranges.append(message.ranges[i])

            json_message = json.dumps(
                dict(type="laser_scan", angle_min=message.angle_min, angle_max=message.angle_max, angle_increment=message.angle_increment,
                     time_increment=message.time_increment, scan_time=message.scan_time, range_min=message.range_min, range_max=message.range_max,
                     ranges=ranges))
            # print(json_message)
            self.interface.send_message("laser_scan", json_message)
        return

    def get_quaternion_from_euler(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def set_interface(self, interface):
        self.interface = interface
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


class SimpleCommander(BasicNavigator):
    def __init__(self):
        super(SimpleCommander, self).__init__()
        self.get_staticmap_srv = self.create_client(
            GetMap, '/map_server/map')
        self.interface = None

    def addMessageQ(self, message):
        self.messageQ.append(message)

    def set_interface(self, interface):
        self.interface = interface
        return

    def requestGoToPose(self, posX, posY, posZ, oriX, oriY, oriZ, oriW):
        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(posX)
        goal_pose.pose.position.y = float(posY)
        goal_pose.pose.orientation.x = float(oriX)
        goal_pose.pose.orientation.y = float(oriY)
        goal_pose.pose.orientation.z = float(oriZ)
        goal_pose.pose.orientation.w = float(oriW)
        i = 0
        self.goToPose(goal_pose)
        while not self.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')
                remainingTime = round(Duration.from_msg(
                    feedback.estimated_time_remaining).nanoseconds / 1e9, 2)
                # remainingTime = (
                #     feedback.estimated_time_remaining).nanoseconds / 1e9

                feedback_msg = 'Estimated time of arrival: ' + \
                    str(remainingTime) + " seconds"
                # print(feedback_msg)
                json_message = json.dumps(
                    dict(type="feedback", feedback=feedback_msg))
                # print(json_message)
                # print(self.interface.ws)
                self.interface.send_message("feedback", json_message)

                # print(json_message)

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
                    goal_pose.pose.position.x = -3.0
                    self.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.getResult()
        json_message = json.dumps(dict())
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            json_message = json.dumps(dict(type="result", result="success"))

        elif result == TaskResult.CANCELED:
            json_message = json.dumps(dict(type="result", result="canceled"))

        elif result == TaskResult.FAILED:
            json_message = json.dumps(dict(type="result", result="failed"))

        else:
            json_message = json.dumps(
                dict(type="result", result="invalid return state"))

        self.interface.send_message("result", json_message)

    def requestStaticMap(self):
        static_map = self.getStaticMap()
        static_map_json = json.dumps(dict(type="static_map", resolution=static_map.info.resolution, width=static_map.info.width,
                                          height=static_map.info.height, positionX=static_map.info.origin.position.x,
                                          positionY=static_map.info.origin.position.y, data=static_map.data.tolist()))
        self.interface.send_message("static_map", static_map_json)

    def requestCancelTask(self):
        self.cancelTask()

    def getStaticMap(self):
        """Get the static map."""
        while not self.get_staticmap_srv.wait_for_service(timeout_sec=2.0):
            self.info('Get static map service not available, waiting...')
        req = GetMap.Request()
        future = self.get_staticmap_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map


def main():

    rclpy.init()
    try:

        # if (messageQ.empty)

        subscriber = SubscriberForWeb()
        simpleCommander = SimpleCommander()
        simpleCommander.waitUntilNav2Active()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(simpleCommander)
        executor.add_node(subscriber)

        web_monitor_interface = WebMonitorInterface(
            subscriber, simpleCommander)
        web_monitor_interface.connect("ws://172.16.165.208:36888")
        try:
            # subscriber_thread = threading.Thread(
            #     target=rclpy.spin, args=(subscriber,))
            # subscriber_thread.start()
            executor_thread = threading.Thread(
                target=executor.spin)
            executor_thread.start()

            commanderThreadPool = ThreadPoolExecutor(max_workers=4)
            while True:
                if not (web_monitor_interface.messageQ.qsize() == 0):
                    message = web_monitor_interface.messageQ.get()
                    commanderThreadPool.submit(
                        on_message, simpleCommander, message)

        except KeyboardInterrupt:
            simpleCommander.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            # commanderThreadPool.shutdown()
            simpleCommander.destroy_node()
            subscriber.destroy_node()
            # executor.shutdown()
    finally:
        rclpy.shutdown()


def on_message(commander, message):
    parsed = json.loads(message)
    # print("ws: received   " + parsed["type"])
    messageType = parsed["type"]
    if (messageType == "requestGoToPose"):
        commander.requestGoToPose(parsed['positionX'], parsed['positionY'], parsed['positionZ'], parsed['orientationX'], parsed['orientationY'],
                                  parsed['orientationZ'], parsed['orientationW'])
    elif (messageType == "requestStaticMap"):
        commander.requestStaticMap()

    elif (messageType == "requestCancelTask"):
        commander.requestCancelTask()


if __name__ == '__main__':
    main()
