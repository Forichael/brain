import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
#import numpy as np
import pygame
#import time
import math
#import sys
#from rdp import rdp
    
class LaserVisualization:
    def __init__(self, model, screen, dpa):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.sub = rospy.Subscriber('/imu/heading', Float32, self.imu_callback)
        self.pub = rospy.Publisher('/wpt/cmd_vel', Float32MultiArray, queue_size = 1)
        scanArray = []
        self.model = model
        self.dpa = dpa
        self.screen = screen
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.zeroIndex = 0
        self.ranges = []
        self.heading = 0
        self.heading_x = 0
        self.heading_y = 0
        self.scaling = 30
        self.tolerance = .1*self.scaling
        self.angle_threshold = .1
        self.cone_detection_angle = .05      #in radians
        self.cone_detection_threshold = .02 #in meters
        self.cone_detection_jump = .3 # in meters
        self.center = (self.model.width/2, self.model.height/2)
        self.heading_point = self.center
        self.cycle = 0
        self.total_cycle = 0
        self.turn_array = [0]*11
        self.vel_array = [0]*11
        self.turn_direction = 0
    def imu_callback(self, scan):
        if(not math.isnan(scan.data)):
            self.heading = scan.data


    def laser_callback(self, scan):
        self.angle_min = scan.angle_min
        self.angle_increment = scan.angle_increment
        self.zeroIndex = -self.angle_min/self.angle_increment
        self.ranges = scan.ranges
        self.draw()
    def draw(self):
        self.screen.fill(pygame.Color('grey'))
        xAndYArray = []
        prev_distance = 0
        max_point_difference = 0
        noise_index_array = []
        temp_index_array = []
        angle_started = False
        temp_angle = self.angle_min


        for i in range(len(self.ranges)):
            #if i in noise_index_array:
                #color = pygame.Color('red')
            #else:
                #color = pygame.Color('blue')
            self.current_angle = i*self.angle_increment + self.angle_min
            if(not math.isnan(self.ranges[i])):
                x = -int(self.ranges[i]*math.sin(self.current_angle)*self.scaling)
                y = -int(self.ranges[i]*math.cos(self.current_angle)*self.scaling)
                if(abs(x) > + self.tolerance and abs(y) > self.tolerance):
                    xAndYArray.append([x + self.model.width/2, y + self.model.height/2])

            pygame.draw.circle(self.screen, pygame.Color('blue'), (x + self.model.width/2,y + self.model.height/2), 2)
        #print self.heading_x
        if(len(xAndYArray) > 0):
            rdp_array = dpa.rdp(xAndYArray, 2)
        #print rdp_array
            line_array = []
            for i in range(len(rdp_array)):
                #print rdp_array[i]
                x = rdp_array[i][0]
                y = rdp_array[i][1]
                pygame.draw.circle(self.screen, pygame.Color('green'), (x,y), 2)
                for j in range(i, len(rdp_array)):
                    x1 = rdp_array[i][0]
                    y1 = rdp_array[i][1]
                    x2 = rdp_array[j][0]
                    y2 = rdp_array[j][1]
                    length = dpa.distance((x1, y1),(x2, y2))
                    if((x2 -x1) != 0 and length < (10*self.scaling)and length > (.5*self.scaling)):
                        slope = (y2-y1)/(x2-x1)
                        line_array.append(Line(x1, y1, x2, y2, self.get_angle((x1, y1), (x2, y2))))
            #for i in renage(len(slope_array)):

            for i, line in enumerate(line_array):
                #line.reset_dots()
                for j, point in enumerate(xAndYArray):
                    distance_to_line = dpa.point_line_distance((point[0], point[1]), (line.x1, line.y1), (line.x2, line.y2))
                    distance_to_start = dpa.distance((point[0], point[1]),(line.x1, line.y1))
                    if (distance_to_line < .14*self.scaling and distance_to_line > 0):
                        #print distance_to_line
                        line.number_of_dots += 1
                        line.total_distance_to_start += distance_to_start
                #print line.number_of_dots
                line_distance = dpa.distance((line.x1, line.y1), (line.x2, line.y2))
                line.line_distance = line_distance
                if(line.number_of_dots > 5 and line_distance/line.number_of_dots < 1.5):
                #(line.number_of_dots > 30 and line.total_distance_to_start/line.number_of_dots < 1.5*self.scaling):
                    pygame.draw.line(self.screen, pygame.Color('red'), (line.x1, line.y1), (line.x2, line.y2), 2)
                    line.is_good_line = True
            number_of_lines = 0
            total = 0
            for i, line, in enumerate(line_array):
                if line.is_good_line:
                    if line.number_of_dots > 0:
                        angle = line.angle
                        while (angle < 0):
                            angle += math.pi
                        while (angle > math.pi):
                            angle -= math.pi
                        total += angle*line.line_distance#/line.number_of_dots
                        #print angle*180.0/math.pi, line.line_distance
                        number_of_lines += 1*line.line_distance#/line.number_of_dots
                    #print angle
            #print 'start'
            average = 0
            if (number_of_lines != 0):
                average = total/number_of_lines
                point3 = ((self.center[0] - math.cos(average)*100), (self.center[1] - math.sin(average)*100))
                pygame.draw.line(self.screen, pygame.Color('green'), self.center, point3, 5)
            if (average != 0):
                if(self.cycle > 5):
                    total_average = self.total_cycle/6
                    if total_average < 0:
                        total_average += math.pi
                    self.turn_direction = total_average - math.pi/2
                    self.current_go_heading = self.turn_direction - self.heading
                    self.turn_array = [0]*11
                    #print self.turn_direction*7/(math.pi/6)
                    index = int(self.turn_direction*7/(math.pi/6))
                    if index < -4:
                        index = -5
                    if index > 4:
                        index = 5
                    self.turn_array[5 + index] = .01
                    msg = Float32MultiArray()
                    self.vel_array = [0]*11
                    total_array = self.vel_array
                    total_array.extend(self.turn_array)
                    msg.data = total_array
                    self.pub.publish(msg)
                    
                    print self.turn_array
                    self.heading_point = ((self.center[0] - math.cos(total_average)*100), (self.center[1] - math.sin(total_average)*100))
                    
                    self.cycle = 0
                    self.total_cycle = 0      
                else:
                    self.cycle += 1
                    self.total_cycle += average
                    pygame.draw.line(self.screen, pygame.Color('black'), self.center, self.heading_point, 5)

        pygame.display.update()

    def get_angle(self, point1, point2):
        return math.atan2((point2[1]-point1[1]),(point2[0]-point1[0]))

class Line(object):
    def __init__(self, x1, y1, x2, y2, angle):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.angle = angle
        self.total_distance_to_start = 0
        self.number_of_dots = 0
        self.is_good_line = False
        self.line_distance = 0
    def add_dots(self):
        self.number_of_dots += 1
    def add_distance(self, distance):
        self.total_distance_to_start += distance
    def reset_dots(self):
        self.number_of_dots = 0
        self.total_distance_to_start = 0

class LidarModel(object):
    def __init__(self):
        self.width = 1000
        self.height = 1000
        self.center = (self.width/2, self.height/2)
        self.size = (self.width, self.height)

class DPAlgorithm():
    def distance(self,  a, b):
        return  math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def point_line_distance(self,  point, start, end):
        if (start == end):
            return self.distance(point, start)
        else:
            n = abs(
                (end[0] - start[0]) * (start[1] - point[1]) - (start[0] - point[0]) * (end[1] - start[1])
            )
            d = math.sqrt(
                (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
            )
            return n / d

    def rdp(self, points, epsilon):
        """
        Reduces a series of points to a simplified version that loses detail, but
        maintains the general shape of the series.
        """
        dmax = 0.0
        index = 0
        i=1
        for i in range(1, len(points) - 1):
            d = self.point_line_distance(points[i], points[0], points[-1])
            if d > dmax :
                index = i
                dmax = d

        if dmax >= epsilon :
            results = self.rdp(points[:index+1], epsilon)[:-1] + self.rdp(points[index:], epsilon)
        else:
            results = [points[0], points[-1]]
        return results


if __name__ == '__main__':
    print 'start'
    rospy.init_node('lidar_visulization')
    model = LidarModel();
    dpa= DPAlgorithm();
    screen = pygame.display.set_mode(model.size)
    laser = LaserVisualization(model, screen, dpa)
    rospy.spin()