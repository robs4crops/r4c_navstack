#!/bin/bash

# Hardcoded JSON file
json_file="farm.json"

# Check if correct number of arguments are provided
if [ $# -lt 2 ]; then
    echo "Usage: $0 <field_number> <line_numbers_comma_separated>"
    exit 1
fi

# Assign the arguments to variables
field_number="$1"  # The field to process
lines="$2"         # The lines (comma-separated list of line IDs)

# Convert lines from comma-separated values to array
IFS=',' read -r -a lines_array <<< "$lines"

# Check if the file exists
if [ ! -f "$json_file" ]; then
    echo "Error: JSON file '$json_file' not found."
    exit 1
fi

# Check if the field exists in the JSON
field_exists=$(jq ".farm.fields[] | select(.name == \"$field_number\")" "$json_file")

if [ -z "$field_exists" ]; then
    echo "Error: Field number $field_number does not exist."
    exit 1
fi

# Check if the 'lines' exist within the selected field
lines_check=$(jq ".farm.fields[] | select(.name == \"$field_number\") | .lines" "$json_file")

if [[ "$lines_check" == "null" ]]; then
    echo "Error: 'lines' field is null or missing in field $field_number."
    exit 1
fi

# Initialize the ROS 2 action goal YAML content
mission_poses=""

# Function to check if a line exists and create poses in alternating A/B order
process_line() {
    line_id="$1"
    index="$2"
    
    # Use jq to find the line with the given ID within the specified field
    line_exists=$(jq ".farm.fields[] | select(.name == \"$field_number\") | .lines[] | select(.id == \"$line_id\")" "$json_file")
    
    if [ -z "$line_exists" ]; then
        echo "Line ID $line_id does not exist in field $field_number."
    else
        # Extract the coordinates for A and B
        lat_A=$(echo "$line_exists" | jq -r '.A.lat // "null"')
        lon_A=$(echo "$line_exists" | jq -r '.A.lon // "null"')
        lat_B=$(echo "$line_exists" | jq -r '.B.lat // "null"')
        lon_B=$(echo "$line_exists" | jq -r '.B.lon // "null"')

        if [[ "$lat_A" == "null" || "$lon_A" == "null" || "$lat_B" == "null" || "$lon_B" == "null" ]]; then
            echo "Error: Missing coordinates for line ID $line_id in field $field_number."
        else
            # Switch between (A -> B) and (B -> A) based on the index
            if (( index % 2 == 0 )); then
                # A -> B
                mission_poses+="
  - pose:
      position:
        x: $lat_A
        y: $lon_A
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - pose:
      position:
        x: $lat_B
        y: $lon_B
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
            else
                # B -> A
                mission_poses+="
  - pose:
      position:
        x: $lat_B
        y: $lon_B
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - pose:
      position:
        x: $lat_A
        y: $lon_A
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
            fi
        fi
    fi
}

# Loop through each provided line ID and alternate the order of A/B coordinates
index=0
for line_id in "${lines_array[@]}"; do
    process_line "$line_id" "$index"
    ((index++))
done

# Construct the YAML content for the ROS 2 action goal
yaml_content=$(printf "mission:\n  header:\n    stamp:\n      sec: 0\n      nanosec: 0\n    frame_id: map\n  poses:%s\n" "$mission_poses")

# Execute the ROS 2 action goal command by passing the YAML content directly
ros2 action send_goal /lspgc/navigation r4c_interfaces/action/Nav "$yaml_content"