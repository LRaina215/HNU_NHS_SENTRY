
'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-30 01:09:37
FilePath: /bubble/src/bubble_contrib/bubble_decision/bubble_decision/gameAction.py
LastEditors: HarryWen
LastEditTime: 2022-08-11 01:29:46
E-mail: robomaster@birdiebot.top
'''
import os
import re
import string
import numpy as np

import rmctrl_msgs.msg
import nav_msgs.msg
import message_filters

from rclpy.node import Node
from game_msgs.msg import GameStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int8
from rcl_interfaces.msg import (
    IntegerRange, ParameterDescriptor, SetParametersResult)


class GameAction():
    def __init__(self, node: Node) -> None:
        self.node = node    
        self.gain_zone_state = 0 #1增益区状态
        self.recover_zone_state = 0 # 2 补给区状态
        self.game_progress = 0 # 3 比赛状态
        self.blood = 400       #4血量
        self.bullet = 750      #5弹量
        self.shoot_heat = 0    #6枪口热量
        
        self.blood_friend1 = 0 #7己方血量1
        self.blood_friend2 = 0 #8己方血量2
        self.blood_friend3 = 0 #9己方血量3
        self.blood_friend4 = 0 #10己方血量4
        
        self.blood_competitor1 = 0 #11敌方血量1
        self.blood_competitor2 = 0 #12敌方血量2
        self.blood_competitor3 = 0 #13敌方血量3
        self.blood_competitor4 = 0 #14敌方血量4
        
        self.remain_time = 300   #15比赛时间
        self.hit_direction = 0   #16受击方向
        # game_status_sub = message_filters.Subscriber(
        #     self, GameStatus, "/status/game")
        # manifold_ctrl_sub = message_filters.Subscriber(
        #     self, Int8, "/status/manifold_ctrl")
        # ts = message_filters.ApproximateTimeSynchronizer(
        #     [game_status_sub, manifold_ctrl_sub], 10, 0.1)
        # ts.registerCallback(self.game_status_callback)

        #self.manifold_ctrl_sub = self.node.create_subscription(
        #    Int8, '/status/manifold_ctrl', self.manifold_ctrl_callback, 10)

        self.game_status_sub = self.node.create_subscription(
            nav_msgs.msg.Odometry, '/game_info', self.game_status_callback, 10)
        
        self.game_initial_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.game_decision_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose', 10)
        
        self.buff_point = {'x': 7.0, 'y': -0.5, 'z': 0.0, 'w': 1.0}   
        self.supply_point = {'x': 0.1, 'y': -0.5, 'z': 0.0,'w' : 1.0} 
        self.current_goal = None
        self.is_beenini = False  
            
    def game_status_callback(self, msg):
        self.gain_zone_state = msg.twist.twist.linear.x    #1增益区状态
        self.recover_zone_state = msg.twist.twist.linear.y #2补给区状态  
        self.game_progess = msg.twist.twist.linear.z       #3比赛状态
        self.blood = msg.twist.twist.angular.x             #4血量
        self.bullet = msg.twist.twist.angular.y            #5弹量
        self.shoot_heat = msg.twist.twist.angular.z        #6枪口热量
        
        self.blood_friend1 = msg.twist.covariance[1]       #7己方血量1
        self.blood_friend2 = msg.twist.covariance[2]       #8己方血量2
        self.blood_friend3 = msg.twist.covariance[3]       #9己方血量3
        self.blood_friend4 = msg.twist.covariance[4]       #10己方血量4
        
        self.blood_competitor1 = msg.twist.covariance[5]   #11敌方血量1
        self.blood_competitor2 = msg.twist.covariance[6]   #12敌方血量2
        self.blood_competitor3 = msg.twist.covariance[7]   #13敌方血量3
        self.blood_competitor4 = msg.twist.covariance[8]   #14敌方血量4
        
        self.remain_time = msg.pose.pose.position.x        #15比赛时间
        self.hit_direction = msg.pose.pose.position.y      #16受击方向   
        
        if self.game_progess == 3:
            self.publish_init(self.supply_point,"supply_point")
        elif self.game_progess == 4:
            if self.blood >= 300:                
                self.publish_goal(self.buff_point,"buff_point")
            elif self.blood < 300:
                self.publish_goal(self.supply_point,"supply_point")    
    
    def publish_init(self,point,name):
        if self.is_beenini:
            return
            
        init = PoseWithCovarianceStamped()
        init.header.stamp = self.node.get_clock().now().to_msg()
        init.header.frame_id = 'map'  
            
        init.pose.pose.position.x = point['x']
        init.pose.pose.position.y = point['y']
        
        init.pose.pose.orientation.z = point['z']
        init.pose.pose.orientation.w = point['w']
        
        self.game_initial_pub.publish(init)
        self.is_beenini = True
        self.node.get_logger().info(f"initialize the pose in {name}")   
        
    def publish_goal(self, point, name):
        if self.current_goal == name:
            return
            
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = 'map' 
        
        goal.pose.position.x = point['x']
        goal.pose.position.y = point['y']
        
        goal.pose.orientation.z = point['z']
        goal.pose.orientation.w = point['w']
        
        self.game_decision_pub.publish(goal)
        self.current_goal = name
        self.node.get_logger().info(f"go to {name}")        
        
    #def manifold_ctrl_callback(self, manifold_ctrl_msg):
    #   self.manifold_ctrl = manifold_ctrl_msg.data

    #def setParamValue(self, topic: string, param: string, value: float):
    #    if self.getParamValue(topic, param) != value:
    #        os.popen(
    #            f". '/home/nvidia/Desktop/bubble/install/setup.sh' && ros2 param set {topic} {param} {value}")
    #
    #def getParamValue(self, topic: string, param: string):
    #    res = os.popen(
    #        f". '/home/nvidia/Desktop/bubble/install/setup.sh' && ros2 param get {topic} {param}")
    #    result = ''.join(re.findall(r"\d+\.?\d*", res.read()))  # 最终设定的参数
    #    return result


class SentryGameAction(GameAction):
    def __init__(self, _node: Node) -> None:
        super().__init__(_node)
        print("sentry down game mode start")

