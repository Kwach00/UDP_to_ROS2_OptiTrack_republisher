# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from tf_transformations import quaternion_from_euler

import socket
import json


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_republisher')
        self.RigidBody = 'CF'
        self.IP = "192.168.110.2"
        self.Port = 12111
        self.publish_string_msg = False
        
        if self.publish_string_msg:
            self.pub_optitrack_string = self.create_publisher(String, 'optitrack_topic_string', 10)
        
        self.pub_optitrack = self.create_publisher(Pose, 'optitrack_topic', 10)
        self.timer_period = 1.0 / 250.0  # 250Hzs
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.serverAddressPort = (self.IP, self.Port)
        self.bufferSize = 1024
        self.UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPClientSocket.bind(self.serverAddressPort)
        
        self.last_json_obj = 0
        

    def timer_callback(self):
        msg_pose = Pose()
        if self.publish_string_msg:
            msgFromServer = String()
        recv_data = self.UDPClientSocket.recvfrom(self.bufferSize)
        json_obj = json.loads(recv_data[0])
        if 'rigid_bodies' in json_obj:
            if self.publish_string_msg:
                msgFromServer.data = json.dumps(json_obj)
                self.pub_optitrack_string.publish(msgFromServer)
            
            if json_obj[self.RigidBody][0] == 1:
                self.last_json_obj = json_obj
            else:
                json_obj = self.last_json_obj
                
            msg_pose.position.x = json_obj[self.RigidBody][1]
            msg_pose.position.y = json_obj[self.RigidBody][2]
            msg_pose.position.z = json_obj[self.RigidBody][3]
            msg_pose.orientation.x = json_obj[self.RigidBody][4]
            msg_pose.orientation.y = json_obj[self.RigidBody][5]
            msg_pose.orientation.z = json_obj[self.RigidBody][6]
            msg_pose.orientation.w = 0.0
            
            self.pub_optitrack.publish(msg_pose)
                            


def main(args=None):
    rclpy.init(args=args)

    optitrack_republisher = MinimalPublisher()

    rclpy.spin(optitrack_republisher)

    optitrack_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
