# map_based_prediction

## Purpose

`map_based_prediction` is a module to predict the future paths of other vehicles and pedestrians according to the shape of the map and the surrounding environment. It also calculates probability of the candidate paths.

## Assumptions
- 対象となる障害物(車や人)の以下の情報がわかっていること
   - カテゴリ(人や車などの種類)
   - 障害部の(重心)位置とその位置にいる時間
   - 物体の位置がmap座標(他の座標系情報しかないときはmap座標への変換式)
- 周辺環境について以下の情報がわかっていること
   - laneletの地図情報


## Known Limits
- 障害物が乗用車、バス、トラックの時
   - 基本的に地図の形状に沿った予測経路を出力
   - 障害物の向きがlaneletの上にいてもそのlaneletとの向きの差が大きいときは地図情報は見ずに予測経路をだす(今回は直線経路)
   - 障害物がどのlaneletにもいないときは地図情報を見ずに予測経路を出す(今回は直線予測)
   - 引かれている経路がは車両のダイナミクスを無視したものを引く可能性がある
- 障害物が人やバイクの時
   - すべての状況において地図情報を見ずに予測経路を出す(今回は直線予測)
- すべての障害物に対して
   - 予測経路はT[s]だけ出てきてなおかつdt[s]分の間隔で出てくる(つまりpredicted pathのサイズNはN=T/dt)
   - Tとdtはパラメータとしてユーザーが決められる
   - 障害物から加速度情報を取ってこれていないので縦方向は等速直線運動を仮定した予測になっている

## Inner-workings / Algorithms

1. Get lanelet path
   The first step is to get the lanelet of the current position of the car. After that, we obtain several trajectories based on the map.

2. Lane Change Detection
   After finding the current lanelet from the current position of the obstacle, our algorithm try to detect the lane change maneuver from the past positions of the obstacle. Our method uses the deviation between the obstacle's current position and its position one second ago and current position to determine if it is about to change lanes. The parameters used for the lane change decision are obtained by analyzing the data obtained from the experiment. We already confirmed that these parameters give the least number of false positives.

3. Confidence calculation
   We use the following metric to compute the distance to a certain lane.

   ```txt
   d = x^T P x
   ```

   where `x=[lateral_dist, yaw_diff]` and `P` are covariance matrices. Therefore confidence values can be computed as

   ```txt
   confidence = 1/d
   ```

   Finally, we normalize the confidence value to make it as probability value. Note that the standard deviation of the lateral distance and yaw difference is given by the user.

4. Drawing predicted trajectories
   From the current position and reference trajectories that we get in the step1, we create predicted trajectories by using Quintic polynomial. Note that, since this algorithm consider lateral and longitudinal motions separately, it sometimes generates dynamically-infeasible trajectories when the vehicle travels at a low speed. To deal with this problem, we only make straight line predictions when the vehicle speed is lower than a certain value (which is given as a parameter).

## Inputs / Outputs

### Input

| Name                                               | Type                                                 | Description                              |
| -------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| `~/perception/object_recognition/tracking/objects` | `autoware_auto_perception_msgs::msg::TrackedObjects` | tracking objects without predicted path. |
| `~/vector_map`                                     | `autoware_auto_mapping_msgs::msg::HADMapBin`         | binary data of Lanelet2 Map.             |

### Output

| Name                     | Type                                                   | Description                           |
| ------------------------ | ------------------------------------------------------ | ------------------------------------- |
| `~/objects`              | `autoware_auto_perception_msgs::msg::PredictedObjects` | tracking objects with predicted path. |
| `~/objects_path_markers` | `visualization_msgs::msg::MarkerArray`                 | marker for visualization.             |

## Parameters

| Parameter                                   | Type   | Description                                                                                                  |
| ------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------ |
| `enable_delay_compensation`                 | bool   | flag to enable the time delay compensation for the position of the object                                    |
| `prediction_time_horizon`                   | double | predict time duration for predicted path [s]                                                                 |
| `prediction_sampling_delta_time`            | double | sampling time for points in predicted path [s]                                                               |
| `min_velocity_for_map_based_prediction`     | double | apply map-based prediction to the objects with higher velocity than this value                               |
| `dist_threshold_for_searching_lanelet`      | double | The threshold of the angle used when searching for the lane to which the object belongs [rad]                |
| `delta_yaw_threshold_for_searching_lanelet` | double | The threshold of the distance used when searching for the lane to which the object belongs [m]               |
| `sigma_lateral_offset`                      | double | Standard deviation for lateral position of objects [m]                                                       |
| `sigma_yaw_angle`                           | double | Standard deviation yaw angle of objects [rad]                                                                |
| `object_buffer_time_length`                 | double | Time span of object history to store the information [s]                                                     |
| `history_time_length`                       | double | Time span of object information used for prediction [s]                                                      |
| `dist_ratio_threshold_to_left_bound`        | double | Conditions for using lane change detection of objects. Distance to the left bound of lanelet.                |
| `dist_ratio_threshold_to_right_bound`       | double | Conditions for using lane change detection of objects. Distance to the right bound of lanelet.               |
| `diff_dist_threshold_to_left_bound`         | double | Conditions for using lane change detection of objects. Differential value of horizontal position of objects. |
| `diff_dist_threshold_to_right_bound`        | double | Conditions for using lane change detection of objects. Differential value of horizontal position of objects. |

## Reference

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363–4369.
