# NVSAM

This project is for demo purposes

## Docker

### Build
```bash
cd nvsam/docker/
docker build . -f ./Dockerfile -t nvsam
```
## How to use?

### Compile nvsam package
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone git@github.com:hhawary/nvsam.git

cd ..
colcon build
```
### Run nvsam

#### Option 1: Online


*Default*

Camera ID 0
```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py
```

Camera ID 1

```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py camera_id:=1
```

set max_distance (default 20.0 meter)

```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py camera_id:=1 max_distance:=10.0
```


#### Option 2: Offline

```
cd ros2_ws
source install/setup.bash
ros2 launch nvsam nvsam_launch.py svo_path:=<path to svo file> max_distance:=10.0
```

