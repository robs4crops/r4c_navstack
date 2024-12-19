import json
import math
import os
import sys
import yaml

import numpy as np
import cv2
import matplotlib.pyplot as plt
# from PIL import Image, ImageDraw

# ------------------------------------------------------------------------------
# Function definitions
# ------------------------------------------------------------------------------

# Define the transformation function to convert lat/lon to xy coordinates


def m_to_px(m):
    return m / pixel_size_m


def to_img_coords(x, y):
    return int(width_px / 2 + m_to_px(x)), int(height_px / 2 - m_to_px(y))


def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    delta_lat = lat_rad - ref_lat_rad
    delta_lon = lon_rad - ref_lon_rad

    earth_radius = 6371000  # approximate value, adjust if needed

    x = earth_radius * delta_lon * math.cos(ref_lat_rad)
    y = earth_radius * delta_lat

    x, y = to_img_coords(x, y)

    # Check if the coordinates are within the bounds of the costmap
    if 0 <= x < width_px and 0 <= y < height_px:
        return x, y
    else:
        print(f"({x}, {y}) is out of the costmap")
        return None  # or raise an exception, depending on your requirements


def draw_headland(headland, datum):
    if not isinstance(headland, dict):
        raise Exception("The 'headland' element must be of type dict")

    if 'id' not in headland:
        raise Exception("The 'id' property is missing in a headland")

    if not isinstance(headland['id'], str):
        raise Exception("The 'id' property in a headland must be of type string")

    if 'points' not in headland:
        raise Exception(f"The property 'points' is missing in headland '{headland['id']}'")

    if not isinstance(headland['points'], list):
        raise Exception(f"The property 'points', in headland {headland['id']}, must be of type list")

    ref_lat = datum[0]
    ref_lon = datum[1]

    converted_headland = []

    for point in headland['points']:
        if not isinstance(point, dict):
            raise Exception(f"A point in a headland must be of type dict")

        if 'lat' not in point:
            raise Exception(f"The property 'lat' is missing in a point of headland {headland['id']}")

        if 'lon' not in point:
            raise Exception(f"The property 'lon' is missing in a point of headland {headland['id']}")

        if not isinstance(point['lat'], float):
            raise Exception(f"The property 'lat', in a point of a headland {headland['id']}, must be of type float")

        if not isinstance(point['lon'], float):
            raise Exception(f"The property 'lon', in a point of a headland {headland['id']}, must be of type float")

        xy_point = latlon_to_xy(point["lat"], point["lon"], ref_lat, ref_lon)

        if xy_point:  # Check the point is not None
            converted_headland.append(xy_point)

    np_converted_points = np.array(converted_headland)
    hull = cv2.convexHull(np_converted_points)

    cv2.fillConvexPoly(Img, hull, (255, 255, 255))

    print("Drawing headland: ", headland)


def draw_line(line, datum):
    if not isinstance(line, dict):
        raise Exception("The 'line' element must be of type dict")

    if 'id' not in line:
        raise Exception("The 'id' property is missing in a line")

    if not isinstance(line['id'], str):
        raise Exception("The 'id' property in a line must be of type string")

    if 'A' not in line:
        raise Exception(f"The property 'A' is missing in line '{line['id']}'")

    if 'B' not in line:
        raise Exception(f"The property 'B' is missing in line '{line['id']}'")

    if not isinstance(line['A'], dict):
        raise Exception(f"The property 'A', in line {line['id']}, must be of type dict")

    if not isinstance(line['B'], dict):
        raise Exception(f"The property 'B', in line {line['id']}, must be of type dict")

    if 'lat' not in line['A']:
        raise Exception(f"The property 'lat' is missing in element 'A' from line {line['id']}'")

    if 'lon' not in line['A']:
        raise Exception(f"The property 'lon' is missing in element 'A' from line {line['id']}'")

    if 'lat' not in line['B']:
        raise Exception(f"The property 'lat' is missing in element 'B' from line {line['id']}'")

    if 'lon' not in line['B']:
        raise Exception(f"The property 'lon' is missing in element 'B' from line {line['id']}'")

    if not isinstance(line['A']['lat'], float):
        raise Exception(f"The property 'lat', in element 'A' from line {line['id']}, must be of type float")

    if not isinstance(line['A']['lon'], float):
        raise Exception(f"The property 'lon', in element 'A' from line {line['id']}, must be of type float")

    if not isinstance(line['B']['lat'], float):
        raise Exception(f"The property 'lat', in element 'B' from line {line['id']}, must be of type float")

    if not isinstance(line['B']['lon'], float):
        raise Exception(f"The property 'lon', in element 'B' from line {line['id']}, must be of type float")

    ref_lat = datum[0]
    ref_lon = datum[1]

    xy_point_A = latlon_to_xy(line['A']['lat'], line['A']['lon'], ref_lat, ref_lon)
    xy_point_B = latlon_to_xy(line['B']['lat'], line['B']['lon'], ref_lat, ref_lon)

    if xy_point_A and xy_point_B:
        cv2.line(Img, xy_point_A, xy_point_B, (255, 255, 255), int(line_thickness_m/pixel_size_m))

    print("Drawing line: ", line)


def draw_obstacle(obstacle, datum):
    if not isinstance(obstacle, dict):
        raise Exception("Obstacle must be of type dict")

    if 'lat' not in obstacle:
        raise Exception("The 'lat' property is missing in obstacle")

    if 'lon' not in obstacle:
        raise Exception("The 'lon' property is missing in obstacle")

    if 'size' not in obstacle:
        raise Exception("The 'size' property is missing in obstacle")

    if not isinstance(obstacle['lat'], float):
        raise Exception("The 'lat' property from an obstacle must be of type float")

    if not isinstance(obstacle['lon'], float):
        raise Exception("The 'lon' property from an obstacle must be of type float")

    if not isinstance(obstacle['size'], float):
        raise Exception("The 'size' property from an obstacle must be of type float")

    ref_lat = datum[0]
    ref_lon = datum[1]
    xy_obstacle = latlon_to_xy(obstacle["lat"], obstacle["lon"], ref_lat, ref_lon)

    size = 0.5 * obstacle["size"]

    if xy_obstacle:
        cv2.circle(Img, xy_obstacle, int(size / pixel_size_m), (0, 0, 0), -1)

        print("Drawing obstacle: ", obstacle)


# ------------------------------------------------------------------------------
# Script execution
# ------------------------------------------------------------------------------

# Check the number of arguments (including the script name itself)
if len(sys.argv) < 3 or len(sys.argv) > 4:
    print(f"Usage: python {sys.argv[0]} <path_to_json_file> <field_id> [<line_thickness_m>]")
    sys.exit(1)  # Exit with a non-zero status to indicate an error

# Get the file path from the argument
json_file_path = sys.argv[1]

# Check if the file exists
if not os.path.isfile(json_file_path):
    print(f"Error: File '{json_file_path}' does not exist.")
    sys.exit(1)

# Check the passed file is a JSON file
try:
    with open(json_file_path, 'r') as json_file:
        farm = json.load(json_file)
        print(f"Successfully loaded JSON file: {json_file_path}")
except json.JSONDecodeError:
    print(f"Error: The file '{json_file_path}' is not a valid JSON file.")
    sys.exit(1)

field_id = sys.argv[2]

# Thickness for lines (m)
line_thickness_m_default = 3.3
line_thickness_m = float(sys.argv[3]) if len(sys.argv) > 3 else line_thickness_m_default

# Size of the image in meters
width_m = 500
height_m = 500

# Size of each pixel in meters
pixel_size_m = 0.1

# Calculate the size of the image in pixels
width_px = int(width_m / pixel_size_m)
height_px = int(height_m / pixel_size_m)

# Create a new white image
Img = np.zeros((width_px, height_px, 3), dtype='uint8')

if 'fields' not in farm:
    raise Exception("The 'fields' property is missing from the farm")
if not isinstance(farm['fields'], list):
    raise Exception("The 'fields' property in the farm must be of type list")

for field in farm['fields']:
    if not isinstance(field, dict):
        raise Exception("Each field in the farm must be of type dict")

    if 'id' not in field:
        raise Exception("The 'id' property is missing from a field")

    if not isinstance(field['id'], str):
        raise Exception("Each field must have an element called 'id' of type string")

    if field['id'] == field_id:
        if 'datum' not in field:
            raise Exception(f"The 'datum' property is missing in field '{field['id']}'")

        if not isinstance(field['datum'], dict):
            raise Exception(f"The 'datum' property, in field '{field['id']}', must be of type dict")

        if 'lat' not in field['datum']:
            raise Exception(f"The 'lat' property is missing from 'datum', in field '{field['id']}'")

        if 'lon' not in field['datum']:
            raise Exception(f"The 'lon' property is missing from 'datum', in field '{field['id']}'")

        if not isinstance(field['datum']['lat'], float):
            raise Exception(f"The 'lat' property from 'datum', in field '{field['id']}', must be of type float")

        if not isinstance(field['datum']['lon'], float):
            raise Exception(f"The 'lon' property from 'datum', in field '{field['id']}', must be of type float")

        datum = field['datum']['lat'], field['datum']['lon']

        if 'lines' not in field:
            raise Exception(f"The 'lines' property is missing in field '{field['id']}'")

        if not isinstance(field['lines'], list):
            raise Exception(f"The 'lines' property, in field '{field['id']}', must be of type list")

        for line in field["lines"]:
            draw_line(line, datum)

        if 'headlands' in field:
            if not isinstance(field['headlands'], list):
                raise Exception(f"The 'headlands' property, in field '{field['id']}', must be of type list")

            for headland in field["headlands"]:
                draw_headland(headland, datum)

        if 'obstacles' in field:
            if not isinstance(field['obstacles'], list):
                raise Exception(f"The 'obstacles' property, in field '{field['id']}', must be of type list")

            for obstacle in field["obstacles"]:
                draw_obstacle(obstacle, datum)

        # plt.imshow(Img, cmap='gray')
        # plt.show()

        response = input("Do you want to save the image? (yes/no): ").lower()

        if response == "yes" or response == "y":
            # Save the image as pgm file.
            gray_image = cv2.cvtColor(Img, cv2.COLOR_BGR2GRAY)
            # Use os.path.expanduser() to translate special symbols like ~, if present.
            # Use os.path.abspath() to expand local paths to absolute paths.
            base_dir = os.path.abspath(os.path.expanduser(os.path.dirname(json_file_path)))
            costmap_filename = "costmap.pgm"
            fully_qualified_costmap_filename = base_dir + "/" + costmap_filename
            cv2.imwrite(fully_qualified_costmap_filename, gray_image)
            print(f"Image saved as {fully_qualified_costmap_filename}")

            # The center of the image is considered as (0, 0), so origin element is computed as
            # shown.
            yaml_description = {"image": costmap_filename, "resolution": pixel_size_m,
                                "origin": [-width_m*0.5, -height_m*0.5, 0.0], "negate": 0,
                                "occupied_thresh": 0.65, "free_thresh": 0.196}

            # Dump dictionary to a yaml file without reordering
            yaml_filename = "costmap.yaml"
            fully_qualified_yaml_filename = base_dir + "/" + yaml_filename

            with open(fully_qualified_yaml_filename, 'w') as f:
                yaml.dump(yaml_description, f, default_flow_style=False, sort_keys=False)

        break
