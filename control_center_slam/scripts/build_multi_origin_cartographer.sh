#!/usr/bin/env bash
# Rebuild only the private mapping engine. Does not start ROS or touch hardware.
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-$(mktemp -d /tmp/control-center-slam-build.XXXXXX)}"
jobs="${BUILD_JOBS:-4}"
mkdir -p "$build_dir/src"
build_dir="$(cd -- "$build_dir" && pwd)"
if [[ -e "$build_dir/src/cartographer" || -e "$build_dir/src/cartographer_ros" ]]; then
  echo 'Use a new empty build directory; existing source will not be overwritten.' >&2
  exit 1
fi
cd "$package_dir/vendor"
sha256sum -c SOURCES.sha256
tar -xzf cartographer-multi-origin-2.0.9004.tar.gz -C "$build_dir/src"
tar -xzf cartographer-ros-2.0.9003.tar.gz -C "$build_dir/src"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cmake -S "$build_dir/src/cartographer" -B "$build_dir/core" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$build_dir/install" \
  -DCARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY=/opt/ros/jazzy/share/cartographer/configuration_files
nice -n 12 cmake --build "$build_dir/core" -j"$jobs" --target \
  cartographer.mapping.2d.multi_origin_range_data_test \
  cartographer.mapping.2d.probability_grid_test \
  cartographer.mapping.internal.2d.insertion_motion_guard_test \
  cartographer.mapping.internal.2d.overlapping_submaps_trimmer_2d_test \
  cartographer_autogenerate_ground_truth cartographer_compute_relations_metrics \
  cartographer_pbstream cartographer_print_configuration
"$build_dir/core/cartographer.mapping.2d.multi_origin_range_data_test"
"$build_dir/core/cartographer.mapping.2d.probability_grid_test"
"$build_dir/core/cartographer.mapping.internal.2d.insertion_motion_guard_test"
"$build_dir/core/cartographer.mapping.internal.2d.overlapping_submaps_trimmer_2d_test"
cmake --install "$build_dir/core"
cmake -S "$build_dir/src/cartographer_ros" -B "$build_dir/ros" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=OFF \
  -Dcartographer_DIR="$build_dir/install/share/cartographer" \
  -DCMAKE_INSTALL_PREFIX="$build_dir/install"
nice -n 12 cmake --build "$build_dir/ros" --target cartographer_node cartographer_occupancy_grid_node -j"$jobs"
for executable in cartographer_node cartographer_occupancy_grid_node; do
  if ldd "$build_dir/ros/$executable" | grep 'not found'; then
    echo 'Runtime dependencies are missing; not publishing the executables.' >&2
    exit 1
  fi
done
mkdir -p "$package_dir/runtime"
install -m 755 "$build_dir/ros/cartographer_node" \
  "$package_dir/runtime/cartographer_multi_origin_node.new"
mv -f "$package_dir/runtime/cartographer_multi_origin_node.new" \
  "$package_dir/runtime/cartographer_multi_origin_node"
install -m 755 "$build_dir/ros/cartographer_occupancy_grid_node" \
  "$package_dir/runtime/cartographer_occupancy_grid_node.new"
mv -f "$package_dir/runtime/cartographer_occupancy_grid_node.new" \
  "$package_dir/runtime/cartographer_occupancy_grid_node"
echo 'Build and tests passed. Rebuild control_center_slam with colcon; restart mapping manually.'
echo "Build logs and sources retained: $build_dir"
