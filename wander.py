#!/usr/bin/env python

import tf
import rospy
import math
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from tf_conversions import transformations


class WanderBot:

  def __init__(self):
    self.map = None
    self.map_origin = None
    self.map_resolution = None
    self.map_width = None
    self.map_height = None
    self.map_zeros = None

    self.traveled_path_set = set()
    self.traveled_path_min_max = None
 
    rospy.init_node('wander')
    self.rate = rospy.Rate(100)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_subscriber_callback)
    while self.map is None:
      self.rate.sleep()

    self.tf_listener = tf.TransformListener()
    while True:
      try:
        self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1))
        break
      except:
        self.rate.sleep()


  def map_subscriber_callback(self, msg):
    compress = 4
    self.map_origin = msg.info.origin.position
    self.map_resolution = msg.info.resolution*compress
    raw_map = np.array(msg.data, dtype=np.int8).reshape(msg.info.width, msg.info.height)
    self.map, self.map_width, self.map_height, self.map_zeros = self.compress_map(raw_map, compress, transposition=True)


  def compress_map(self, raw_map, compress=2, transposition=True):
    m = int(len(raw_map)/compress)
    n = int(len(raw_map[0])/compress)
    zero_cnt = 0
    res = np.zeros([m, n], dtype=np.int8)
    for i in range(m):
      for j in range(n):
        k = -1
        for ii in range(compress):
          for jj in range(compress):
            k = max(raw_map[i*compress+ii][j*compress+jj], k)
        if k == 0:
          zero_cnt += 1
        res[i][j] = np.int8(k)
    if transposition:
      res, m, n = res.T, n, m
    return res, m, n, zero_cnt


  def get_pos(self):
    try:
      (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      euler = transformations.euler_from_quaternion(rot)
      x = int((trans[0] - self.map_origin.x) / self.map_resolution)
      y = int((trans[1] - self.map_origin.y) / self.map_resolution)
      th = euler[2]
      return (x, y, th)
    except:
      return None


  def traveled_path(self, pos):
    if pos is not None:
      x, y = pos[0], pos[1]
      self.traveled_path_set.add((x, y))
      if self.traveled_path_min_max is None:
        self.traveled_path_min_max = (x, x, y, y)
      else:
        x_min, x_max, y_min, y_max = self.traveled_path_min_max
        x_min = min(x, x_min)
        x_max = max(x, x_max)
        y_min = min(y, y_min)
        y_max = max(y, y_max)
        self.traveled_path_min_max = (x_min, x_max, y_min, y_max)


  def find_next_travel_path(self):
    pos = self.get_pos()
    if pos is None:
      return None

    self.traveled_path(pos)
    visit_list = []
    visit_list.append((pos[0], pos[1]))
    visit_parent = {}
    visit_parent[(pos[0], pos[1])] = None

    while len(visit_list) > 0:
      next_list = []
      target_list = []
      for x, y in visit_list:
        for dx, dy in [(1,0), (0,-1), (-1,0), (0,1)]:
        #for dx, dy in [(1,0), (1,-1), (0,-1), (-1,-1), (-1,0), (-1,1), (0,1), (1,1)]:
          nx, ny = x+dx, y+dy
          if 0 <= nx < self.map_width and 0 <= ny < self.map_height \
              and self.map[nx][ny] == 0 and (nx, ny) not in visit_parent:
            next_list.append((nx, ny))
            visit_parent[(nx, ny)] = (x, y)
            if (nx, ny) not in self.traveled_path_set:
              target_list.append((nx, ny))

      if len(target_list) > 0:
        scores = []
        for x, y in target_list:
          x_min, x_max, y_min, y_max = self.traveled_path_min_max
          x_min = min(x, x_min)
          x_max = max(x, x_max)
          y_min = min(y, y_min)
          y_max = max(y, y_max)
          scores.append(abs(x_max-x_min-y_max+y_min))
        target = target_list[scores.index(max(scores))]
        path = []
        while visit_parent[target] is not None:
          path.append(target)
          target = visit_parent[target]
        return path
      else:
        visit_list = next_list

    return None


  def find_path(self, pos, target):
    visit_list = []
    visit_list.append((pos[0], pos[1]))
    visit_parent = {}
    visit_parent[(pos[0], pos[1])] = None

    while len(visit_list) > 0:
      next_list = []
      for x, y in visit_list:
        for dx, dy in [(1,0), (0,-1), (-1,0), (0,1)]:
        #for dx, dy in [(1,0), (1,-1), (0,-1), (-1,-1), (-1,0), (-1,1), (0,1), (1,1)]:
          nx, ny = x+dx, y+dy
          if nx == target[0] and ny == target[1]:
            visit_parent[(nx, ny)] = (x, y)
            path = []
            p = (nx, ny)
            while visit_parent[p] is not None:
              path.append(p)
              p = visit_parent[p]
            return path
          if 0 <= nx < self.map_width and 0 <= ny < self.map_height \
              and self.map[nx][ny] == 0 and (nx, ny) not in visit_parent:
            next_list.append((nx, ny))
            visit_parent[(nx, ny)] = (x, y)

      visit_list = next_list

    return None


  def stop(self):
    if not rospy.is_shutdown():
      twist = Twist()
      self.cmd_vel_pub.publish(twist)


  def forward(self, mps=1):
    if not rospy.is_shutdown():
      twist = Twist()
      twist.linear.x = math.fabs(mps)
      self.cmd_vel_pub.publish(twist)


  def backward(self, mps=-1):
    if not rospy.is_shutdown():
      twist = Twist()
      twist.linear.x = -math.fabs(mps)
      self.cmd_vel_pub.publish(twist)


  def rotate(self, th):
    start_rotate = False
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
      if rospy.Time.now() > start_time + rospy.Duration(10):
        return None

      pos = self.get_pos()
      if pos is None:
        return False

      diff = th - pos[2]
      if diff < 0:
        diff += 2*math.pi
      if diff < 0.02*math.pi or diff > 1.98*math.pi:
        if start_rotate:
          self.stop()
        return True

      start_rotate = True
      twist = Twist()
      if diff < math.pi:
        twist.angular.z = math.pi / 4
      else:
        twist.angular.z = -math.pi / 4
      
      self.cmd_vel_pub.publish(twist)
      self.rate.sleep()
    return False


  def angle(self, pos, target):
    th = 0
    if target[0] == pos[0]:
      if target[1] > pos[1]:
        th = 0.5*math.pi
      else:
        th = -0.5*math.pi
    else:
      th = math.atan((target[1] - pos[1]) / (target[0] - pos[0]))
      if target[0] < pos[0]:
        if target[1] < pos[1]:
          th -= math.pi
        else:
          th += math.pi
    return th


  def move(self, path):
    print(f'path={path}')
    while not rospy.is_shutdown() and path is not None and len(path) > 0:
      next_pos = path.pop()
      start_time = rospy.Time.now()
      while not rospy.is_shutdown():
        if rospy.Time.now() > start_time + rospy.Duration(30):
          self.traveled_path(next_pos)
          while len(path) > 0:
            self.traveled_path(path.pop())
          self.stop()
          break

        pos = self.get_pos()
        if pos is None:
          self.stop()
          self.rate.sleep()
          continue

        if pos[0] == next_pos[0] and pos[1] == next_pos[1]:
          self.traveled_path(pos)
          self.stop()
          break
        
        th = self.angle(pos, next_pos)
        rotated = self.rotate(th)
        if rotated is None:
          if random.random() < 0.7:
            self.backward(0.2)
          else:
            self.forward(0.2)
        elif rotated is True:
          if (pos[0], pos[1]) in self.traveled_path_set and (next_pos[0], next_pos[1]) in self.traveled_path_set:
            self.forward(0.3)
          else:
            self.forward(0.1)
  
        #print(f'pos={pos} next_pos={next_pos} th={th}')
        self.rate.sleep()


  def travel(self, ratio=0.9, timeout=None):
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
      if timeout is not None and rospy.Time.now() > start_time + rospy.Duration(timeout):
        break
      path = self.find_next_travel_path()
      if path is None:
        self.rate.sleep()
      else:
        self.move(path)
        print(f'has traveled {len(self.traveled_path_set)} of {self.map_zeros} nodes')
        if len(self.traveled_path_set) > ratio * self.map_zeros:
          break
    return (rospy.Time.now() - start_time) / rospy.Duration(1)


  def goto(self, target):
    while not rospy.is_shutdown():
      pos = self.get_pos()
      if pos[0] == target[0] and pos[1] == target[1]:
        if len(target) > 2:
          self.rotate(target[2])
        break
      path = self.find_path(pos, target)
      if path is None:
        self.travel(10)
      else:
        self.move(path)


if __name__ == '__main__':
  wander_bot = WanderBot()
  print('wander_bot has inited OK ...')
  home_pos = wander_bot.get_pos()
  print('start traveling ...')
  travel_time = wander_bot.travel(ratio=0.95)
  print(f'finish traveling, time={travel_time}s, now go back home {home_pos} ...')
  wander_bot.goto(home_pos)
  print('wander_bot shut down ...')

