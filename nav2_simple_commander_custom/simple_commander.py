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

import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander_custom.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


import websocket
import threading
import json
from nav_msgs.srv import GetMap


def on_open(ws):
    ws.send("Connected")
    print("Opened connection")


def on_message(ws, message):
    print(message)
    parsed = json.loads(message)
    print(parsed['positionX'], parsed['positionY'], parsed['positionZ'], parsed['orientationX'], parsed['orientationY'],
          parsed['orientationZ'], parsed['orientationW'])

    goToPose(ws, parsed['positionX'], parsed['positionY'], parsed['positionZ'], parsed['orientationX'], parsed['orientationY'],
             parsed['orientationZ'], parsed['orientationW'])


def on_close(ws):
    print("close")


"""
Basic navigation demo to go to pose.
"""


class SimpleCommander(BasicNavigator):
    def __init__(self):
        super(SimpleCommander, self).__init__()
        self.get_staticmap_srv = self.create_client(
            GetMap, '/map_server/map')

    def requestGoToPose(self, ws, posX, posY, posZ, oriX, oriY, oriZ, oriW):

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = super.get_clock().now().to_msg()
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
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
                    goal_pose.pose.position.x = -3.0
                    self.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            ws.send('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            ws.send('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            ws.send('Goal failed!')
        else:
            print('Goal has an invalid return status!')
            ws.send('Goal has an invalid return status!')

    def sendStaticMap(self, ws):
        static_map = self.getStaticMap()
        static_map_json = json.dumps(dict(type="static_map", width=static_map.info.width,
                                          height=static_map.info.height, positionX=static_map.info.origin.position.x,
                                          positionY=static_map.info.origin.position.y, data=static_map.data.tolist()))
        # ws.send(static_map_json)
        print(static_map_json)

        # temp = json.loads(static_map_json)
        # print(temp["type"])
        # print(temp["data"])
        # print(len(temp["data"]))
        ##### add#####

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
    simpleCommander = SimpleCommander()
    simpleCommander.waitUntilNav2Active()
    rclpy.spin(simpleCommander)
    while rclpy.ok():
        print("ok")
        pass

    # run simple commander
    # ws
    # run wst
    # run spin(subscribe pos)

    # exit key interrupt

    # simpleCommander.sendStaticMap(0)

    # rclpy.spin(simpleCommander)
    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.x = 0.707
    # initial_pose.pose.orientation.w = -0.707
    # navigator.setInitialPose(initial_pose)
    # print('init_pose', initial_pose.pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()
    # sendStaticMap(0)
    # rclpy.spin
    # static_map = navigator.getStaticMap()
    # print("width : ", static_map.info.width)
    # print("height : ", static_map.info.height)
    # print("position : ", static_map.info.origin.position)
    # static_map_json = json.dumps(dict(type="static_map", width=static_map.info.width,
    #                                   height=static_map.info.height, positionX=static_map.info.origin.position.x,
    #                                   positionY=static_map.info.origin.position.y, data=static_map.data))
    # static_map_json = dict(type="static_map", width=static_map.info.width,
    #                        height=static_map.info.height, positionX=static_map.info.origin.position.x,
    #                        positionY=static_map.info.origin.position.y, data=static_map.data.tolist())
    # print(static_map.data.tolist())
    # print(static_map_json)
    # print("orientation : ", static_map.info.origin.orientation)
    # print("data size : ", len(static_map.data))
    # print("----")
    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')
    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    # Go to our demos first goal pose
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = 2.0
    # goal_pose.pose.position.y = -0.5
    # goal_pose.pose.orientation.w = 1.0
    # # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    # navigator.goToPose(goal_pose)
    # print('goal pose  ', goal_pose.pose)
####
    # websocket.enableTrace(True)
    # print("h1")
    # ws = websocket.WebSocketApp(
    #     "ws://172.16.165.127:30555", on_message=on_message, on_close=on_close, on_open=on_open)
    # wst = threading.Thread(target=ws.run_forever)
    # print("h2")
    # wst.daemon = True
    # print("h3")
    # wst.start()
    # print("h4")
    # while True:
    #     time.sleep(10)
####
    # i = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################
    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')
    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()
    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)
    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')
    # navigator.lifecycleShutdown()
    # exit(0)
if __name__ == '__main__':
    main()
