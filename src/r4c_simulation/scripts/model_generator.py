#!/usr/bin/env python3
import shutil
import os
import argparse
import random

parser = argparse.ArgumentParser("Gazebo world generator")
parser.add_argument("model_name", help="Name of the model to be inserted to the world")
parser.add_argument("x_number", help="Number of object to be inserted along axis x", default=1)
parser.add_argument("-x_separation", help="Separation of objects along axis x", default=1)
args, unknown = parser.parse_known_args()

model_name = str(args.model_name)

row_initial_coordinates = [0.0, 0.0]
row_temp_coordinates = row_initial_coordinates[0]
step_size = float(args.x_separation)

x_number = int(args.x_number)

shutil.copyfile("templates/model_template.txt", "copy_model_template.txt")

model_idx = 0
for idx in range(x_number):
    temp_orientation = random.uniform(0,3.1415)
    temp_model_name = str(model_name)+"_"+str(model_idx)
    model_definition_template = "\n    <include>"+"\n      <name>"+str(temp_model_name)+"</name>"+"\n      <pose>"+str(row_temp_coordinates) + " 0 0 0 0 "+str(temp_orientation)+" </pose>\n       <uri>model://"+str(model_name)+"</uri>\n    </include>"
    row_temp_coordinates = row_temp_coordinates + step_size
    with open("copy_model_template.txt", "a") as f:
        f.write(model_definition_template)
    model_idx+=1

with open("copy_model_template.txt", "a") as f:
        f.write("\n  </model>\n</sdf>")

os.rename("copy_model_template.txt","model.sdf")
