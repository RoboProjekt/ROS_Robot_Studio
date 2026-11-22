#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=ros-robot-studio/humble-usb-cameras:0.3.0
model="${SO101_STEREO_MODEL:-}"
if [[ -z "$model" ]]; then
    case "${SO101_STEREO_PROFILE:-}" in
        high|low)
            model="$root/checkpoints/so101-${SO101_STEREO_PROFILE}-height-fisheye/compact-stereo.onnx"
            ;;
        *)
            printf 'Set SO101_STEREO_PROFILE=high (42–60 cm) or low (10–20 cm), or set SO101_STEREO_MODEL.\n' >&2
            exit 1
            ;;
    esac
fi
stream="$root/src/ros_robot_studio/vision/stream.py"

camera_device() {
    local node
    if ! node="$(readlink -e -- "$1")" || [[ ! "$node" =~ ^/dev/video[0-9]+$ || ! -c "$node" ]]; then
        printf 'USB camera not found at %s\n' "$1" >&2
        return 1
    fi
    printf '%s\n' "$node"
}

camera_1="$(camera_device "${1:-/dev/v4l/by-path/pci-0000:01:00.0-usb-0:5:1.0-video-index0}")"
camera_2="$(camera_device "${2:-/dev/v4l/by-path/pci-0000:01:00.0-usb-0:4:1.0-video-index0}")"
if [[ "$camera_1" == "$camera_2" ]]; then
    printf 'Choose a separate video device for each camera.\n' >&2
    exit 1
fi
if [[ ! -f "$model" ]]; then
    printf 'Stereo policy not found at %s\n' "$model" >&2
    exit 1
fi

if ! docker image inspect "$image" >/dev/null 2>&1; then
    docker build --tag "$image" --file "$root/docker/Dockerfile.usb-cameras-humble" "$root/docker"
fi

exec docker run --rm --init --sig-proxy=true --name studio-so101-usb-cameras --network host \
    --device "$camera_1:$camera_1:rw" --device "$camera_2:$camera_2:rw" \
    --group-add "$(stat -c %g "$camera_1")" --group-add "$(stat -c %g "$camera_2")" \
    --mount "type=bind,source=$(realpath -- "$model"),target=/opt/studio/compact-stereo.onnx,readonly" \
    --mount "type=bind,source=$stream,target=/opt/studio/camera_stream.py,readonly" \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23}" --env ROS_LOCALHOST_ONLY=0 \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env FASTRTPS_DEFAULT_PROFILES_FILE=/opt/studio/fastdds_localhost.xml \
    "$image" /opt/stereo-venv/bin/python /opt/studio/so101_stereo_topics.py \
    "$camera_1" "$camera_2" /opt/studio/compact-stereo.onnx
