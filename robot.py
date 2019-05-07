from math import sin, cos, sqrt, pow, pi, atan2
import random
import shapely
import matplotlib.patches as patches
import copy
from random import gauss
from shapely.geometry import LineString
from shapely.geometry import MultiPolygon
from util import *


class Robot:

    def __init__(self, world_size, landmarks):
        self.world_size = world_size
        self.landmarks = landmarks
        self.landmark_size = 6
        self.x = random.random() * world_size[0]
        self.y = random.random() * world_size[1]
        self.theta = random.random() * 2.0 * pi

        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.range_noise = 0.0
        self.bearing_noise = 0.0

    def set_state(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.theta = float(new_orientation % (2.0*pi))

    def set_noise(self, new_f_noise, new_t_noise, new_r_noise, new_b_noise):
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.range_noise = float(new_r_noise)
        self.bearing_noise = float(new_b_noise)

    def isValid(self):
        if self.x < 0 or self.y < 0 or self.x > self.world_size[0] or self.y > self.world_size[1]:
            return False
        for o in self.landmarks:
            length = self.landmark_size/2
            if o[0]-length<self.x<o[0]+length:
                return False
            if o[1]-length<self.y<o[1]+length:
                return False

        return True

    # TODO: Moves a robot
    def move(self, Z):
        # delta_theta = random.random() * 2.0 * pi
        delta_theta = gauss(0, sqrt(self.turn_noise))
        obs_landmarks = []
        for i in range(int(len(Z) / 2)):
            r = Z[2 * i]
            phi = Z[2 * i + 1]
            theta_rad = phi + self.theta
            # print(i+1)
            obs_x = self.x + r * cos(theta_rad)
            obs_y = self.y + r * sin(theta_rad)
            obs_landmarks.append((obs_x, obs_y))
        # print(obs_landmarks)
        obs_obstacles, edges = self.get_obstacles(obs_landmarks)

        delta_theta = self.detect_collision(delta_theta, obs_obstacles, edges)
        new_theta = self.theta + delta_theta
        delta_x = 10 * cos(new_theta)
        delta_y = 10 * sin(new_theta)
        new_x = self.x + delta_x + gauss(0, sqrt(self.forward_noise))
        new_y = self.y + delta_y + gauss(0, sqrt(self.forward_noise))
        
        self.set_state(new_x, new_y, new_theta)

        return (delta_x, delta_y, delta_theta)
    
    def get_obstacles(self, landmarks):
        obstacles = []
        # edges = [[(0, 0), (0, 250)], [(0, 0), (250, 0)], [(250, 0), (250, 250)], [(0, 250), (250, 250)]]
        edges = []
        for l in landmarks:
            left_bot = (l[0] - self.landmark_size / 2, l[1] - self.landmark_size / 2)
            left_top = (l[0] - self.landmark_size / 2, l[1] + self.landmark_size / 2)
            right_bot = (l[0] + self.landmark_size / 2, l[1] - self.landmark_size / 2)
            right_top = (l[0] + self.landmark_size / 2, l[1] + self.landmark_size / 2)
            # if self.x > l[0] - self.landmark_size / 2 and self.x < l[0] + self.landmark_size / 2:
            #     if self.y > l[1] - self.landmark_size / 2 and self.y < l[1] + self.landmark_size / 2:
            #         print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            edges.append([left_bot, left_top])
            edges.append([left_bot, right_bot])
            edges.append([right_bot, right_top])
            edges.append([left_top, right_top])
            obstacle = []
            obstacle.append(left_bot)
            obstacle.append(right_bot)
            obstacle.append(left_top)
            obstacle.append(right_top)
            obstacles.append([copy.copy(obstacle), []])
        return obstacles, edges
    
    def detect_collision(self, delta_theta, obs_obstacles, edges):
        stuck = False
        potential_next = (self.x + 10 * cos(self.theta + delta_theta), self.y + 10 * sin(self.theta + delta_theta))
        potential_line = [(self.x, self.y), potential_next]
        not_intersect = True
        not_hit_wall = True
        for e in edges:
            if isIntersect(potential_line, e):
                not_intersect = False
        if potential_next[0] < 0 or potential_next[0] > self.world_size[0] or potential_next[1] < 0 or potential_next[1] > self.world_size[1]:
            not_hit_wall = False
        cnt = 0
        while not (not_intersect and not_hit_wall) and not stuck:
            delta_theta += pi/6 - (2 * pi if delta_theta > pi  else 0)
            # print(self.x)
            # print(self.y)
            # print(delta_theta)
            potential_next = (self.x + 10 * cos(self.theta + delta_theta), self.y + 10 * sin(self.theta + delta_theta))
            # print(potential_next)
            potential_line = [(self.x, self.y), potential_next]
            # print(potential_line)
            intersect = False
            not_hit_wall = False
            # print("Edges\t", edges)
            # print("P line\t", potential_line)
            for e in edges:
                if isIntersect(potential_line, e):
                    intersect = True
            if intersect:
                cnt += 1
            if cnt == 13:
                # print("Stuck")
                stuck = True
                    # print("Intersect")
            if potential_next[0] > 0 and potential_next[0] < self.world_size[0] and potential_next[1] > 0 and potential_next[1] < self.world_size[1]:
                not_hit_wall = True
        # not_hit_wall = True
        # not_hit_obst = True
        # if potential_next[0] < 0 or potential_next[0] > self.world_size[0] or potential_next[1] < 0 or potential_next[1] > self.world_size[1]:
        #     not_hit_wall = False
        # if LineString([(self.x, self.y), potential_next]).intersects(MultiPolygon(obs_obstacles)):
        #     not_hit_obst = False
        # while not (not_hit_wall and not_hit_obst):
        #     delta_theta += pi/6 - (2 * pi if delta_theta > pi  else 0)
        #     potential_next = (self.x + 10 * cos(self.theta + delta_theta), self.y + 10 * sin(self.theta + delta_theta))
        #     if potential_next[0] > 0 and potential_next[0] < self.world_size[0] and potential_next[1] > 0 and potential_next[1] < self.world_size[1]:
        #         not_hit_wall = True
        #     if not LineString([(self.x, self.y), potential_next]).intersects(MultiPolygon(obs_obstacles)):
        #         not_hit_obst = True

        return delta_theta

    # TODO: Returns a list of range bearing measurements
    def sense(self):
        Z = []
        for landmark in self.landmarks:
            other_landmarks = copy.copy(self.landmarks)
            other_landmarks.remove(landmark)
            # print(other_landmarks)
            other_obstacles, other_edges = self.get_obstacles(other_landmarks)
            if not LineString([(self.x, self.y), landmark]).intersects(MultiPolygon(other_obstacles)):

                r = sqrt(pow(landmark[0] - self.x, 2) + pow(landmark[1] - self.y, 2)) + gauss(0, sqrt(self.range_noise))
                # r = sqrt(pow(landmark[0] - self.x, 2) + pow(landmark[1] - self.y, 2))
                Z.append(r)
                # theta_rad = atan2(landmark[1] - self.y, landmark[0] - self.x)
                # print((theta_rad/pi*180) + (0 if theta_rad > 0  else 360))
                phi = atan2(landmark[1] - self.y, landmark[0] - self.x) - self.theta + gauss(0, sqrt(self.bearing_noise))
                # phi = atan2(landmark[1] - self.y, landmark[0] - self.x) - self.theta
                Z.append(phi)
            # else:
            #     Z.append(-1)
            #     Z.append(-1)
        # print(len(Z))
        return Z
    
    def sense_exp(self):
        Z = []
        for landmark in self.landmarks:
            r = sqrt(pow(landmark[0] - self.x, 2) + pow(landmark[1] - self.y, 2))
            Z.append(r)
            phi = atan2(landmark[1] - self.y, landmark[0] - self.x) - self.theta
            Z.append(phi)
        return Z

            

    # TODO: Move a particle according to the provided input vector
    def move_particle(self, dx, dy, dth):
        self.x += dx + gauss(0, sqrt(self.forward_noise))
        self.y += dy + gauss(0, sqrt(self.forward_noise))
        self.theta += dth + gauss(0, sqrt(self.turn_noise))
        self.set_state(self.x, self.y, self.theta)