#!/bin/bash

################################################################################################################################################
# The purpose of this script is to build package files for the MIP SDK in a docker image of an OS for an arch
################################################################################################################################################
set -e

# Get some arguments from the user
os="ubuntu"
arch="amd64"
build_docs=false
while [[ $# -gt 0 ]]; do
  case $1 in
    --os)
      os="$2"
      shift # past argument
      shift # past value
      ;;
    --arch)
      arch="$2"
      shift # past argument
      shift # past value
      ;;
    --docs)
      build_docs=true
      shift # past argument
      ;;
    *)
      shift # past argument
      ;;
  esac
done

# Find the script directory
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
project_dir="${script_dir}/.."
docker_project_dir="/home/microstrain/mipsdk"
dockerfile="${script_dir}/Dockerfile.${os}"
image_name="microstrain/mipsdk_${os}_builder:${arch}"


# Build the docker image
docker build \
  -t "${image_name}" \
  --build-arg ARCH="${arch}" \
  --build-arg USER_ID="$(id -u)" \
  --build-arg GROUP_ID="$(id -g)" \
  -f "${dockerfile}" \
  "${project_dir}"

# Different flags for run depending on if we are running in jenkins or not
if [ "${ISHUDSONBUILD}" != "True" ]; then
  docker_it_flags="-it"
fi

if [ ${build_docs} = true ]; then
  build_dir_name="build_docs"
  configure_flags="\
  -DMICROSTRAIN_BUILD_DOCUMENTATION=ON \
  -DMICROSTRAIN_BUILD_DOCUMENTATION_QUIET=OFF \
  -DCMAKE_BUILD_TYPE=RELEASE"
  build_target="package_docs"
else
  build_dir_name="build_${os}_${arch}"
  configure_flags="\
    -DMICROSTRAIN_BUILD_EXAMPLES=ON \
    -DMICROSTRAIN_BUILD_PACKAGE=ON \
    -DCMAKE_BUILD_TYPE=RELEASE"
  build_target="package"
fi

# Run the build in the docker image
docker run \
  --rm \
  ${docker_it_flags} \
  --entrypoint="/bin/bash" \
  -v "${project_dir}:${docker_project_dir}" \
  -w "${docker_project_dir}" \
  --user="microstrain" \
  "${image_name}" -c " \
    rm -rf ${docker_project_dir}/${build_dir_name}; \
    mkdir ${docker_project_dir}/${build_dir_name}; \
    cd ${docker_project_dir}/${build_dir_name}; \
    cmake ${docker_project_dir} ${configure_flags}; \
    cmake --build . -j$(nproc); \
    cmake --build . --target ${build_target}; \
  "
