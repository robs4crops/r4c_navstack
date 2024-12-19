#!/bin/bash

script_name="$(basename "${BASH_SOURCE[0]}")"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"

ros_distros=("noetic" "foxy" "humble")

usage() {
    echo "Usage: ${script_name} <options>"
    echo
    echo "Options:"
    echo
    echo "  -h          - Show this message."
    echo "  -l          - Use the base ros_dev image from this machine. If no flag, use the base ros_dev image from Eurecat's image registry"
    echo "  -n          - Do not use cache when building the Docker image"
    echo "  -u          - Push the Docker image to Eurecat's Docker image registry"
    echo "  -v version  - The target version of ROS. Required input. Available ROS distros: ${ros_distros[@]}"
    echo
    exit 155
}

ros_distro=""
cache=""
push="false"
use_local_ros_dev_image="false"

if [ ${#} -eq 0 ]
then
  echo "No input arguments provided"
  usage
fi

# Process input from user
while getopts 'hlnuv:' option
do
  case "${option}" in
    h)
      usage
      ;;
    l)
      use_local_ros_dev_image="true"
      ;;
    n)
      cache="--no-cache"
      ;;
    u) # p(u)sh
      push=true
      ;;
    v)
      ros_distro="${OPTARG}"
      ;;
    ?)
      usage
      ;;
  esac
done

if [ -z ${ros_distro} ]
then
  echo "No ROS distro provided" 1>&2
  exit 1
# Check if the ROS distro introduced by the user is valid.
elif [[ ! " ${ros_distros[*]} " =~ " ${ros_distro} " ]]
  then
  echo "Invalid ROS distro provided. Allowed ROS distros: ${ros_distros[@]}" 1>&2
  exit 1
fi

docker_file="${script_dir}/Dockerfile.dev"
docker_image_registry="gitlab.local.eurecat.org:5050/robotics-automation"

source_repository="robotics-dockers"
docker_source_image="ros_dev:${ros_distro}"

if [ "${use_local_ros_dev_image}" = "true" ];then
  docker_base_image="${docker_source_image}"
  pull=""
else
  docker_base_image="${docker_image_registry}/${source_repository}/${docker_source_image}"
  pull="--pull" # --pull always attempt to pull a newer version of the image
fi

docker_image_name="robs4crops:${ros_distro}"
#context_dir="$(cd "${script_dir}/../" > /dev/null && pwd)"

DOCKER_BUILDKIT=1 docker image build -f "${docker_file}" ${cache} ${pull} \
  --progress=plain \
  --build-arg docker_base_image="${docker_base_image}" \
  --build-arg docker_image_building_timestamp="$(date --utc '+%Y-%m-%d-%H-%M-%S')" \
  --tag "${docker_image_name}" "${script_dir}"

if [ "${push}" = "true" ];then
  destination_repository="robs4crops"
  target_image="${docker_image_registry}/${destination_repository}/${docker_image_name}"
  docker image tag "${docker_image_name}" "${target_image}"
  docker push "${target_image}"
  docker image rm "${target_image}"
fi
