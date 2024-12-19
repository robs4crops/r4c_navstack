#!/usr/bin/env bash

# Function to check if a line exists and create poses in alternating A/B order
process_line() {
    local field_id="${1}"
    local line_id="${2}"
    local index="${3}"

    poses=""

    # Use jq to find the line with the given ID within the specified field
    line_exists="$(jq ".fields[] | select(.id == \"${field_id}\") | .lines[] | select(.id == \"${line_id}\")" "${json_file}")"

    if [ -z "${line_exists}" ]; then
        echo "Line id '${line_id}' does not exist in field '${field_id}', in the farm described in the file '${json_file}'" >&2 # Print in std error channel
    else
        # Extract the coordinates for A and B
        lat_A="$(echo "${line_exists}" | jq -r '.A.lat')"
        lon_A="$(echo "${line_exists}" | jq -r '.A.lon')"
        lat_B="$(echo "${line_exists}" | jq -r '.B.lat')"
        lon_B="$(echo "${line_exists}" | jq -r '.B.lon')"

        # echo "A: (${lat_A}, ${lon_A}), B: (${lat_B}, ${lon_B})"

        if [ -z "${lat_A}" ] || [ -z "${lon_A}" ] || [ -z "${lat_B}" ] || [ -z "${lon_B}" ]; then
            echo "Error: Missing coordinates for line '${line_id}' in field '${field_id}' in the file '${json_file}'" >&2 # Print in std error channel
        else
            # Switch between (A -> B) and (B -> A) based on the index
            if (( index % 2 == 0 )); then
                # A -> B
                poses+="
  - pose:
      position:
        x: ${lat_A}
        y: ${lon_A}
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - pose:
      position:
        x: ${lat_B}
        y: ${lon_B}
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
            else
                # B -> A
                poses+="
  - pose:
      position:
        x: ${lat_B}
        y: ${lon_B}
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - pose:
      position:
        x: ${lat_A}
        y: ${lon_A}
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
            fi
        fi
    fi

    echo "${poses}"
}

script_name="$(basename "${BASH_SOURCE[0]}")"
script_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd -P)"

# Check if correct number of arguments are provided
if [ ${#} -lt 4 ]; then
    echo "Usage: ${script_name} <farm_json_file> <field_id> <comma-separated_line_numbers> <mission_prefix>"
    echo "Allowed mission prefixes: lsps, lspg, lspgc, lspsim"
    exit 1
fi

json_file="$(realpath ${1})" # Farm file. Expand to full path in case a local path is passed
field_id="${2}"              # The field to process
line_ids="${3}"              # The lines (comma-separated list of line IDs)
mission_prefix="$(echo "${4}" | tr '[:upper:]' '[:lower:]')" # Get mission prefix in lower case

# Check if the mission prefix is allowed.
if [ "${mission_prefix}" != "lsps" ] && [ "${mission_prefix}" != "lspg" ] && [ "${mission_prefix}" != "lspgc" ] && [ "${mission_prefix}" != "lspsim" ];then
    echo "Error: The mission prefix must be one of the following: lsps, lspg, lspgc, lspsim"
    exit 1
fi

# Check if the file exists.
if [ ! -e "${json_file}" ]; then
    echo "Error: File '${json_file}' not found."
    exit 1
fi

if ! jq -e 'has("fields")' "${json_file}" > /dev/null; then
    echo "Error: The 'fields' element does not exist in the farm described in the file '${json_file}'"
    exit 1
fi

# Check if all elements in the fields array have the 'id' property.
if ! jq -e 'all(.fields[]; has("id"))' "$json_file" > /dev/null; then
    echo "Error: Not all fields have the property 'id' in the farm described in the file '${json_file}'"
    exit 1
fi

# Check if the indicated field exists in the JSON
if ! jq -e ".fields[] | select(.id == \"${field_id}\")" "${json_file}" > /dev/null; then
    echo "Error: Field '${field_id}' does not exist in the farm described in the file '${json_file}'"
    exit 1
fi

# Check if the 'lines' element exist within the selected field
if ! jq -e ".fields[] | select(.id == \"${field_id}\") | .lines" "${json_file}" > /dev/null; then
    echo "Error: Lines element does not exist in the field '${field_id}' in the file '${json_file}'"
    exit 1
fi

# Convert lines from comma-separated values to array
IFS=',' read -r -a line_id_array <<< "${line_ids}"

# Loop through each provided line ID and alternate the order of A/B coordinates
index=0

mission_poses=""

for line_id in "${line_id_array[@]}"; do
    poses="$(process_line "${field_id}" "${line_id}" "${index}")"
    [ -z "${poses}" ] && exit 1 # If there is an error when processing the line, then exit.
    mission_poses+="${poses}"
    ((index++))
done

frame_id="map"

# Construct the msg for the ROS 2 action goal
msg="$(printf "mission:\n  header:\n    stamp:\n      sec: 0\n      nanosec: 0\n    frame_id: ${frame_id}\n  poses:%s\n" "${mission_poses}")"

#echo -e "${msg}"

# Execute the ROS 2 action goal command by passing the YAML content directly
ros2 action send_goal /${mission_prefix}/navigation r4c_interfaces/action/Nav "${msg}"