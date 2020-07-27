import pygame
import math
import numpy as np
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy


def inverse_kin(orgin, target):
    orgin = orgin
    upper = 90 #.09 m
    lower = 162 #.162 m

    dis = np.subtract(target, orgin)
    r = np.sqrt(np.sum(np.multiply(dis, dis)))
    r_lim = upper+lower
    if r > r_lim:
        r = r_lim

    angle = math.atan2(*dis)

    cos_param = ((upper**2) + (r**2) - (lower**2))/(2.0*upper*r)
    if cos_param < -1.0:
        print("r is to small to find valid gamma value")
        gamma = math.pi
    elif cos_param > 1.0:
        print("r is to large to find a valid gamma value")
        gamma = 0
    else:
        gamma = math.acos(cos_param)

    return angle, gamma, r, upper, lower


def sim_path(stride, amp_high, amp_low, duration, time):
    n_time = math.fmod(time, duration*2)

    if n_time < duration:
        pos_y = math.sin((math.pi)*(1/duration)*n_time)*amp_high
        pos_x = stride*(1/duration)*n_time
    else:
        n_time = duration - (n_time - duration)
        pos_y = math.sin((math.pi) * (1 / duration) * n_time) * amp_low
        pos_x = stride * (1 / duration) * n_time
    return [pos_x, pos_y]

WIDTH = 500
HEIGHT = 500
FPS = 30

# Define Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

## initialize pygame and create window
pygame.init()
pygame.mixer.init()  ## For sound
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("<Your game>")
clock = pygame.time.Clock()  ## For syncing the FPS

orgin = [int(WIDTH/2), int(HEIGHT/2)]

class WVIZ_Node(Node):
    def __init__(self):
        super().__init__('wviz_node')
        
        #Setting up subscriptions
        self.subscription = self.create_subscription(
            Twist,
            'leg_pose',
            self.dolly_cmd_callback,
            0)
        self.subscription  # prevent unused variable warning
        
        self.subscription_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            0)
        self.subscription_joy  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Twist, 'dolly_cmd', 0)
        self.publish_timer = self.create_timer((1/200), self.publish_dolly_cmd)

        self.pos_p_offset = [0.0, .2]
        self.left_pos_p_offset = [0.0, .2]
        
        self.draw_timer = self.create_timer((1/200), self.draw)
        self.cur_time = 1 #time.time()
        self.front = 0
        self.back = 0
        
        self.floor_offset_y = 200
        self.floor_offset_x = 0
        self.time_scale = 0
        self.stride_scale_x = 0
        self.stride_scale_y = 0

    def dolly_cmd_callback(self, msg):
        self.front = msg.angular.x
        self.back = msg.angular.y
        
    def joy_callback(self, msg):
        y = msg.axes[4]
        x = msg.axes[3]
        
        
        
        v = (msg.axes[5] - 1)/-2
        b = (msg.axes[2] - 1)/-2
        
        self.time_scale = (v)/50
        if v > 0:
            self.stride_scale_x = msg.axes[1]
            self.stride_scale_y = b
        else:
            self.stride_scale_x = 0
            self.stride_scale_y = 0
        
        self.floor_offset_y = 190 + (y*50)
        self.floor_offset_x = x*70

        
    def publish_dolly_cmd(self):
        msg_out = Twist()
        msg_out.linear = Vector3(x=-self.pos_p_offset[0] , y=self.pos_p_offset[1], z=0.0)
        msg_out.angular = Vector3(x=self.left_pos_p_offset[0], y=self.left_pos_p_offset[1], z=0.0)
        self.publisher_.publish(msg_out)
        
    def draw(self):
        if self.time_scale == 0:
            n_time = math.fmod(self.cur_time, 1*2)
            if (n_time - 1) > 0.005 or (n_time - 1) < -0.005 :
                self.cur_time = self.cur_time + .0075
        self.cur_time = self.cur_time + self.time_scale
        cur_time = self.cur_time

        # 3 Draw/render
        screen.fill(WHITE)

        mouse_g = pygame.mouse.get_pos()

        pygame.draw.circle(screen, BLACK, orgin, 5)

        pos = sim_path((100*self.stride_scale_x), (-80*self.stride_scale_y), 0, 1, cur_time)
        left_pos = sim_path((100*self.stride_scale_x), (-80*self.stride_scale_y), 0, 1, cur_time+1)

        self.pos_p_offset = np.divide(np.add(pos, [self.floor_offset_x-(50*self.stride_scale_x), self.floor_offset_y]), [1000,1000])
        self.left_pos_p_offset = np.divide(np.add(left_pos, [self.floor_offset_x-(50*self.stride_scale_x), self.floor_offset_y]), [1000,1000])

        n_pos = np.add(pos, np.add(orgin, [self.floor_offset_x-(30*self.stride_scale_x), self.floor_offset_y]))
        angle, gamma, r, upper, lower = inverse_kin(orgin, n_pos)
        point = np.add(orgin, [math.sin(angle)*r, math.cos(angle)*r])
        point_a = np.add(orgin, [math.sin(angle+gamma)*upper, math.cos(angle+gamma)*upper])
        point_b = np.add(orgin, [math.sin(angle-gamma)*upper, math.cos(angle-gamma)*upper])

        pygame.draw.line(screen, BLACK, (int(point_a[0]), int(point_a[1])), (int(orgin[0]), int(orgin[1])), 5)
        pygame.draw.line(screen, BLACK, (int(point_b[0]), int(point_b[1])), (int(orgin[0]), int(orgin[1])), 5)
        pygame.draw.line(screen, BLACK, (int(point_a[0]), int(point_a[1])), (int(point[0]), int(point[1])), 5)
        pygame.draw.line(screen, BLACK, (int(point_b[0]), int(point_b[1])), (int(point[0]), int(point[1])), 5)
        
        upper = 90 #.09 m
        lower = 162 #.162 m
        
        gamma = (self.back - self.front)/2
        angle = self.back - gamma 
        r = (math.cos(gamma)*upper) + math.sqrt(lower**2 - (math.sin(gamma)*upper) **2)
        point = np.add(orgin, [math.sin(angle)*r, math.cos(angle)*r])
        point_a = np.add(orgin, [math.sin(angle+gamma)*upper, math.cos(angle+gamma)*upper])
        point_b = np.add(orgin, [math.sin(angle-gamma)*upper, math.cos(angle-gamma)*upper])

        pygame.draw.line(screen, BLUE, (int(point_a[0]), int(point_a[1])), (int(orgin[0]), int(orgin[1])), 5)
        pygame.draw.line(screen, BLUE, (int(point_b[0]), int(point_b[1])), (int(orgin[0]), int(orgin[1])), 5)
        pygame.draw.line(screen, BLUE, (int(point_a[0]), int(point_a[1])), (int(point[0]), int(point[1])), 5)
        pygame.draw.line(screen, BLUE, (int(point_b[0]), int(point_b[1])), (int(point[0]), int(point[1])), 5)
    
        pygame.display.flip()
       
def main(args=None):
    rclpy.init(args=args)

    wviz_node = WVIZ_Node()

    rclpy.spin(wviz_node)

    wviz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
