# NICP-Based Localization
In this project, your task is to develop an NICP-Based Localizer system compliant with the ROS environment.

Localization is the task of estimating the pose of a sensor given its current measurement and the map of the environment. In the _planar_ case, if we have a reasonable initial guess of the sensor pose, it is possible to solve a Least Squares problem to refine the latter (otherwise, a multimodal  distribution has to be estimated, see __Probabilistic Robotics__ exam!).

To keep it short, `we can try alining the measurement on the map using the initial guess provided by the user`

To complete the project, you have to develop a node that:
- Listens to a `nav_msgs/OccupancyGrid`
  - Populate a KD-tree with the world coordinates of the __occupied cells__
- Listen to `/initialpose` topic to get the __Initial Guess__
- Listen to `sensor_msgs/LaserScan` messages
  - Agument the scans with normals information using the `NormalEstimator`
  - Compute the transform between the current scan and the map using NICP

Along with the localizer, you will need to start a `map_server` to publish the world map and `stage_ros` to simulate the robot moving in the map. Refeer to the `Requirements` section to get more informations.

Your task is to complete the `NormalLocalizer2D` class (located in `include/normal_localizer2d.h` and `src/normal_localizer2d.cpp`), the `NormalEstimator` class (located in `include/nicp/normal_estimator.h` and `src/nicp/normal_estimator.cpp`), the `NICP` class (located in `include/nicp/eigen_nicp_2d.h` and `src/nicp/eigen_nicp_2d.cpp`) and the main node (located in `bin/localizer_node.cpp`).

**IMPORTANT**: If you don't implement a really nice prediction (see later), NormalICP will fail localizing on some parts of the map. This is expected due to a resolution mismatch between the scan coming from stage and the predicted scan using the kd-tree. __We do not expect you to fix this issue__, however if you want, try visualizing the prediction (using OpenCV or RViz) and implement a more suitable prediction function (maybe with raycasting on the map, or using SDF functions).

# Obstacle Registration
Before starting the localizer, the node should receive both the map and its metadata from ROS. We implemented for you a `Map` class (located in `include/map.h`) that already parse these informations for you.

It provides informations regarding the size (rows and columns) of the map and easy accessors.

An occupancy grid, assigns a value at every cell of the map, namely:
  - **Occupied** : the cell represent a wall or a mapped object.
  - **Free** : the cell represents a free, travearsable area.
  - **Unknown** : the cell represents unknown or unmapped area.

These values are enumerated as a `CellType` (located in `include/map.h`).

- When an occupancy grid is received by your node, initialize the global _Map_ object, and pass it to the localizer.

- Complete the `NormalLocalizer2D::setMap` method, which explores the map and register all obstacles into a KD-Tree.

# Initial Pose
Using `RViz`, you can initialize the localizer by setting an initial estimate for the localizer. To do so, subscribe to the `/initialpose` topic and update the localizer pose based on the messages received (`NormalLocalizer2D::setInitialPose`)

# Normal ICP
One of the major pitfalls of P2P ICP (point-to-point) is the number of local minima solutions caused by wrong correspondences computed during the alignment process. The cause relates to the nearest neighbors approach used to compute the correspondences, and the sparsity of the scan itself.

Integrating surface information during the alignment process, allows the moving cloud to "slide" along planar surfaces thus removing most of the local minima constrained solutions.

We define a new point type `PointNormal2f` containing:
1. Point coordinates (x, y)
2. Normal vector (z, w)

The normals are computed by the `NormalEstimator` (located in `include/nicp/normal_estimator.h`) which takes an input vector of points (`std::vector<Eigen::Vector2f>`) and output a vector of PointNormal2f (`std::vector<PointNormal2f>`).

Furthermore, the ICP module should be slightly adjusted to handle `PointNormal2f` types. We provided some initial definitions in the `include/nicp/eigen_nicp_2d.h` header.

# Normal Estimation
To augment a scan using normals, you have to implement the `NormalEstimator` class (located at `include/nicp/normal_estimator.h` and `src/nicp/normal_estimator.cpp`).
The procedure for generating the normals is the following:

- Instantiate a kd-tree on the input cloud
- for every point in the input cloud, search its neighbors 
- skip points that have less than 3 neighbors
- compute the mean and covariance of the neighbors
- compute the eigenvectors of the covariance matrix
- pick the smallest eigenvector (first column) as normal vector
- Verify that normal vector is pointing towards the origin (see pseudocode)
- push in the output cloud the point with normal (create a Vector4f point)

This procedure can be described with the following pseudocode:

```txt
Instantiate a kd_tree on input_cloud
output_cloud.clear()
For p in input_cloud:
  neighbors_of_p = kd_tree.fastSearch(p)
  if size(neighbors_of_p) < 3:
    continue
  mean, cov = computeMeanAndCovariance(neighbors_of_p)
  eigenvectors_of_cov = getEigenVectors(cov)
  normal_of_p = eigenvectors_of_cov.columns(0)
  if (-p.dot(normal_of_p) < 0):
    normal_of_p = -normal_of_p
  point_with_normal = Vector4f();
  point_with_normal << p, normal_of_p
  output_cloud.push_back(point_with_normal)
```

You can use the `computeMeanAndCovariance` (found in `include/icp/eigen_covariance.h`) to compute the covariance matrix and the `Eigen::SelfAdjointEigenSolver` class to get the eigenvectors (see [documentation](https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html#a837627aecb3ba7ed40a2e1bfa3806d08))

We also provide the `normal_viewer` node (found in `bin/normal_viewer.cpp`) to verify that the normals are correctly computed.


# Localization
Once a scan and a sufficiently close initial pose estimate is given, the localizer can process the data and attempt localization (`NormalLocalizer2D::process`):
- Generate a scan prediction taken at the initial pose and augment it with normals (`NormalLocalizer2D::getPrediction`)
- Set the NICP solver initial guess equal to the initial pose.
- Run NICP between the input scan and the prediction scan.
- Update the current pose of the laser based on the solver solution.

To generate the prediction, remember to also load range and angular parameters of the input scan to the localizer (`NormalLocalizer2D::setLaserParams`).

After the input scan is processed, remember to send a TF message (`geometry_msgs::TransformStamped`) using the `tf2_ros::TransformBroadcaster` object, along with a `nav_msgs::Odometry` message in the `/odom_out` topic.

You can test your localizer using `RViz`. We have provided a configuration that you can directly run after your node has started.

Go to the project directory, source ros and launch rviz with our configuration
```sh
rviz -d test_data/rviz.rviz
```

## Requirements


### Map Server
- Install the `ros-${DISTRO}-map-server` package. In our case (valid for Lattinone VM) we are using _ROS Noetic_
   ```sh
    sudo apt install ros-noetic-map-server
   ```
- To launch the node, go to the project directory, source ros and launch the `map_server` node
  - ```sh
    source /opt/ros/noetic/setup.bash
    rosrun map_server map_server test_data/cappero_map.yaml    
    ```

### Stage-ROS
- Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package. As above, we are using _ROS Noetic_
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```
- To launch the simulator, go to the project directory, source ros, launch the roscore and launch the `stageros` node
  - ```sh
    rosrun stage_ros stageros test_data/cappero.world
    ```

- You should see the simulator running on a dedicated window. The interfaces are ROS-based and the topics of interest for us are:
  - `/base_scan` : Contains laser scanner data from the robot (Output)
  - `/cmdvel` : Robot commands (Input)
  
## General TIPS
- If you have doubts on how to link your package in the ROS environment, take a look at the `rp_10_scan_matcher` exercise and take a look at its README.
