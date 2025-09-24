# PointCloud to LaserScan Converter
This node converts 3D point clouds (PointCloud2) to 2D laser scans (LaserScan) with configurable height, angle, and range filtering. It applies a 3D transform to points before projection. This node is cross-compatible with ROS1 and ROS2 (the build automatically detects the ROS version).

## Dependencies

- **ROS**: [ROS1 Noetic](http://wiki.ros.org/noetic/Installation) or [ROS2 Humble/Iron](https://docs.ros.org/en/rolling/Installation.html)
- **System Packages**:
    ```bash
    # Ubuntu/Debian
    sudo apt install build-essential cmake libgoogle-glog-dev libgflags-dev \
                     liblua5.1-0-dev libeigen3-dev
    ```

## Setup

1. Clone this repository.

2. **For ROS1**: Add to your `~/.bashrc`:
    ```bash
    echo "export ROS_PACKAGE_PATH=$(pwd):\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

3. **For ROS2**: Add to your `~/.bashrc`:
    ```bash
    echo "export AMENT_PREFIX_PATH=$(pwd)/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Build:
    ```bash
    make
    ```

## Usage

### Configuration

Edit `config/highbeams.lua` to configure:
- **range_min/max**: Distance filtering (m)
- **angle_min/max**: Angular range (rad) 
- **height_min/max**: Z-axis filtering (m)
- **num_ranges**: Number of laser scan rays
- **Topics**: Input PointCloud2 and output LaserScan topic names

### Run

```bash
# Start the converter
./bin/pointcloud_to_laserscan --config=config/highbeams.lua

# With verbose output
./bin/pointcloud_to_laserscan --config=config/highbeams.lua --v=1
```