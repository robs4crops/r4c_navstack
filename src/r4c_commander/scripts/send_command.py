#!/usr/bin/env python3

import json
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from r4c_interfaces.action import Nav

class NavActionClient(Node):

    def __init__(self):
        super().__init__('r4c_nav_action_client')
        self._action_client = ActionClient(self, Nav, '/lspsim/navigation')

    def send_goal(self, coordinates):
        
        goal_msg = Nav.Goal()
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        for coord in coordinates:
            pose = PoseStamped()
            pose.pose.position.x = coord[1]  # latitude as x
            pose.pose.position.y = coord[0]  # longitude as y
            path_msg.poses.append(pose)

        goal_msg.mission = path_msg

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

# Function to extract coordinates for given feature IDs
def extract_coordinates(input_ids, geojson_file):
    # Load GeoJSON data
    with open(geojson_file, 'r') as f:
        data = json.load(f)
    
    # Dictionary to store coordinates for given IDs
    coordinates_dict = {}
    
    # Loop through features
    for feature in data['features']:
        # Check if feature ID matches any of the input IDs
        if str(feature['properties']['id']) in input_ids:
            # Extract coordinates
            coordinates = feature['geometry']['coordinates']
            coordinates_dict[str(feature['properties']['id'])] = coordinates
    
    return coordinates_dict

def main(args=None):
    rclpy.init(args=args)
    action_client = NavActionClient()

    if len(sys.argv) < 3:
        print("Usage: python script.py <input_ids> <geojson_file>")
        sys.exit(1)

    input_ids = sys.argv[1].split(',')
    geojson_file = sys.argv[2]

    coordinates_dict = extract_coordinates(input_ids, geojson_file)

    coordinates_lists = []
    print("Sending R4C mission goal for:")

    # Print coordinates for each specified ID
    for i, id_ in enumerate(input_ids, start=1):
        print(f"Line {id_}:")
        coordinates = coordinates_dict[id_]
        # Invert coordinates for even IDs
        if i % 2 == 0:
            coordinates = list(reversed(coordinates))
        coordinates_lists.extend(coordinates)
        for coordinate in coordinates:
            print(coordinate)

    future = action_client.send_goal(coordinates_lists)
    print("Mission sent!")

    rclpy.spin_until_future_complete(action_client, future)

if __name__ == "__main__":
    main()

