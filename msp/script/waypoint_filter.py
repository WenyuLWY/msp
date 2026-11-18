#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TARE /way_point 方向去抖节点。

问题
----
TARE 的 lookahead 在 1 Hz 规划周期里会偶尔翻到反方向（实测两视点相距 15.4 m），
local_planner 的 joyDir 随之翻转约 180 度，机器人反复掉头。

关键点：TARE 发布的是 **carrot（前视点）**，不是 goal。
        它的位置本来就该随机器人移动，只有 **方向** 是有意义的。
        所以本节点只锁方向、不锁点 —— 距离沿用 TARE 给的值，点继续跟着机器人走。
        （早期版本锁的是点，结果机器人去追一个过期位置，到了才发现目标已经跑到背后。）

做法
----
每帧把新 waypoint 换成从机器人出发的单位方向 u：

  dang = angle(已锁方向, u)

  dang < ang_tol                -> 直接跟随，锁方向更新为 u
  dang >= ang_tol               -> 视为“想换方向”，进入待确认状态：
      待确认方向连续保持 confirm_time 秒，才真正切换
      否则输出 robot + 已锁方向 * r（方向锁住，距离跟随 TARE）

安全阀
------
  机器人连续 stuck_time 秒没有位移 -> 放行待确认方向（避免卡死）

用法
----
  python3 waypoint_filter.py

  给 local_planner 的 localPlanner 节点加一行 remap：
      <remap from="way_point" to="/way_point_filtered"/>

参数
----
  ~ang_tol_deg     90.0   小于这个角度直接跟随（只挡回头，不挡转弯）
  ~pend_tol_deg    25.0   待确认方向的容差，超出即重新计时
  ~confirm_time     2.0   反向请求需要持续多久才认
  ~stuck_time       5.0   机器人多久没位移就放行
  ~stuck_eps        0.3   位移判定的最小距离
  ~output_topic    /way_point_filtered
"""

import math

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


def angle_between(a, b):
    d = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1]))
    return math.acos(d)


class WaypointFilter(object):
    def __init__(self):
        self.ang_tol = math.radians(rospy.get_param('~ang_tol_deg', 90.0))
        self.pend_tol = math.radians(rospy.get_param('~pend_tol_deg', 25.0))
        self.confirm_time = rospy.get_param('~confirm_time', 2.0)
        self.stuck_time = rospy.get_param('~stuck_time', 5.0)
        self.stuck_eps = rospy.get_param('~stuck_eps', 0.3)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        out_topic = rospy.get_param('~output_topic', '/way_point_filtered')

        self.robot = None          # (x, y)
        self.dir = None            # 已锁定的单位方向
        self.pending = None        # 待确认的单位方向
        self.pending_t = 0.0
        self.last_pos = None       # 用于判定位移
        self.last_pos_t = 0.0

        self.pub = rospy.Publisher(out_topic, PointStamped, queue_size=2)
        rospy.Subscriber('/state_estimation', Odometry, self.odom_cb, queue_size=2)
        rospy.Subscriber('/way_point', PointStamped, self.wp_cb, queue_size=2)

        rospy.loginfo('[wp_filter] ang_tol=%.0f deg  pend_tol=%.0f deg  confirm=%.1f s  '
                      'stuck=%.1f s  -> %s',
                      math.degrees(self.ang_tol), math.degrees(self.pend_tol),
                      self.confirm_time, self.stuck_time, out_topic)

    def odom_cb(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        now = rospy.Time.now().to_sec()
        if self.last_pos is None:
            self.last_pos = self.robot
            self.last_pos_t = now
        elif math.hypot(self.robot[0] - self.last_pos[0],
                        self.robot[1] - self.last_pos[1]) > self.stuck_eps:
            self.last_pos = self.robot
            self.last_pos_t = now

    def wp_cb(self, msg):
        now = rospy.Time.now().to_sec()
        x, y, z = msg.point.x, msg.point.y, msg.point.z

        if self.robot is None:
            self.publish(x, y, z)
            return

        dx = x - self.robot[0]
        dy = y - self.robot[1]
        r = math.hypot(dx, dy)

        # 目标就在脚下（TARE reset waypoint），直接透传，并清空方向锁
        if r < 1e-3:
            self.dir = None
            self.publish(x, y, z)
            return

        u = (dx / r, dy / r)

        if self.dir is None:
            self.dir = u
            self.pending = u
            self.pending_t = now
            rospy.loginfo('[wp_filter] lock  (%.1f deg)  <- first', math.degrees(math.atan2(u[1], u[0])))
            self.publish(x, y, z)
            return

        dang = angle_between(self.dir, u)

        if dang < self.ang_tol:
            self.dir = u
            self.pending = u
            self.pending_t = now
            self.publish(x, y, z)
            return

        # 大方向变化：进入待确认
        if angle_between(self.pending, u) > self.pend_tol:
            self.pending = u
            self.pending_t = now

        stuck = (now - self.last_pos_t) > self.stuck_time
        waited = now - self.pending_t

        if waited >= self.confirm_time or stuck:
            self.dir = u
            self.pending = u
            self.pending_t = now
            rospy.loginfo('[wp_filter] switch (%.0f deg, waited %.1fs%s)',
                          math.degrees(dang), waited, ', stuck' if stuck else '')
            self.publish(x, y, z)
            return

        # 方向锁住，距离跟随 TARE
        hx = self.robot[0] + self.dir[0] * r
        hy = self.robot[1] + self.dir[1] * r
        rospy.logwarn_throttle(
            1.0, '[wp_filter] hold  d_ang=%.0f deg  r=%.1fm  (waited %.1f/%.1fs)',
            math.degrees(dang), r, waited, self.confirm_time)
        self.publish(hx, hy, z)

    def publish(self, x, y, z):
        out = PointStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.point.x = x
        out.point.y = y
        out.point.z = z
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('waypoint_filter')
    WaypointFilter()
    rospy.spin()
