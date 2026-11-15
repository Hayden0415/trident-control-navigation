#!/usr/bin/env bash
# Start the assignment container with local X11 authentication for Gazebo/RViz.
set -euo pipefail
image="${TRIDENT_IMAGE:-trident-control-course:2026}"
if [[ -z "${DISPLAY:-}" ]]; then
  echo '未检测到 DISPLAY。请在 Linux 图形桌面终端中运行本脚本。' >&2
  exit 1
fi
if ! command -v xauth >/dev/null; then
  echo '宿主机需要 xauth：sudo apt-get install xauth' >&2
  exit 1
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
  echo "缺少镜像 $image，请先按 README 构建。" >&2
  exit 1
fi
auth_dir="$(mktemp -d -t trident-xauth.XXXXXXXX)"
trap 'rm -rf "$auth_dir"' EXIT
: > "$auth_dir/authority"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$auth_dir/authority" nmerge -
if [[ ! -s "$auth_dir/authority" ]]; then
  echo '无法取得当前 DISPLAY 的 Xauthority，请在已登录的本地图形桌面中重试。' >&2
  exit 1
fi
chmod 600 "$auth_dir/authority"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$script_dir/gpu_args.py" > "$auth_dir/gpu.args"
extra_args=()
if [[ "${TRIDENT_DEV:-0}" == "1" ]]; then
  source_dir="$(cd -- "$script_dir/../src" && pwd)"
  extra_args+=(--mount "type=bind,src=$source_dir,dst=/opt/nav_ws/src")
fi
mapfile -d '' -t gpu_args < "$auth_dir/gpu.args"
docker run --rm -it --init --name trident-course --shm-size 1g \
  -e ROS_DOMAIN_ID=76 -e ROS_LOCALHOST_ONLY=1 \
  -e DISPLAY -e XAUTHORITY=/tmp/course.xauth -e QT_QPA_PLATFORM=xcb \
  -e QT_X11_NO_MITSHM=1 "${gpu_args[@]}" \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$auth_dir/authority,dst=/tmp/course.xauth,readonly" \
  "${extra_args[@]}" "$image" bash
