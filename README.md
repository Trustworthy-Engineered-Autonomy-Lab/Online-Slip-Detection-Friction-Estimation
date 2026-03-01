# Online Slip Detection and Friction Coefficient Estimation

**Purpose:**
This repo contains a ROS2 node that detects slip (drift) events and estimates a representative friction coefficient (mu) from IMU + odometry data for experiments used in the associated paper.

**Key file:**
- `src/drift_detector/drift_detector/revised_detector.py` — ROS2 Node that:
  - Subscribes to: /ackermann_cmd (AckermannDriveStamped), /odom (Odometry), /sensors/imu/raw (Imu), /odometry/filtered (Odometry)
  - Publishes: Bool on topic `is_drifting`
  - Computes a slip metric from the difference between filtered and raw odometry magnitudes
  - During detected drift windows computes `mu = sqrt(ax^2 + ay^2) / az` using IMU linear acceleration
  - Buffers per-event mu values and prints/stores per-run summaries

**How detection works:**
- If `slip_estimate > linear_threshold` for longer than drift_length, node enters drifting window
- While drifting, mu samples are appended: `mu = sqrt(ax^2 + ay^2) / az`
- On the drift end, the node records the maximum mu from the event (used by downstream analysis)

**Runtime outputs for parsing:**
- The node prints arrays at shutdown:
  - Times: list of drift start timestamps (REMOVE_DRIFT_TIMES)
  - Mus: list of recorded mu values per event (REMOVE_DRIFT_MUS)
These prints are intended for downstream parsing and k-fold analysis.

**Requirements**

* Ubuntu 20.04
* Python 3.8+
'''bash
pip install -r requirements.txt
'''
* ROS 2 Foxy
* F1Tenth stack – clone and build the ROS workspace as described in the
  [F1Tenth getting‑started guide](https://f1tenth.readthedocs.io/en/stable/getting_started/firmware/drive_workspace.html#doc-drive-workspace):
  ```bash
  # create a drive workspace and clone the system repository
  mkdir -p ~/f1tenth_ws/src
  cd ~/f1tenth_ws/src
  git clone https://github.com/f1tenth/f1tenth_system.git
  # then `colcon build` from the workspace root, source the install/setup.bash, etc.

**Dataset:**
- `Data/Mu&OffsetData.xlsx` - an Excel file with all collected runs.
