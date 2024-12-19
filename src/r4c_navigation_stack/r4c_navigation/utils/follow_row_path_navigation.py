#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from r4c_navigation.action import PathNavigation


class PathNavigationActionClient(Node):

    def __init__(self):
        super().__init__('follow_row_path_navigation')
        self._action_client = ActionClient(self, PathNavigation, 'path_navigation')

    def send_goal(self):
        self.get_logger().info('Sending follow row action of type path_navigation')
        goal_msg = PathNavigation.Goal()
        goal_msg.global_plan_topic = "central_line"
        goal_msg.controller_id = "FollowPath"
        goal_msg.path_type = 0
        goal_msg.path_dir = 1
        goal_msg.nav_type = 0
        goal_msg.check_goal_reached = True

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = PathNavigationActionClient()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()