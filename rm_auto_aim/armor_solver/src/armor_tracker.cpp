// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_tracker.hpp"
// std
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim
{
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff),
  detect_count_(0),
  lost_count_(0),
  last_yaw_(0) {}

void Tracker::init(const Armors::SharedPtr & armors_msg) noexcept
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  FYT_INFO("armor_solver", "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  if (tracked_armor.type == "large" &&
    (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
  {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::update(const Armors::SharedPtr & armors_msg) noexcept
{
  // // KF predict
  // Eigen::VectorXd ekf_prediction = ekf->predict();

  // bool matched = false;
  // // Use KF prediction as default target state if no matched armor is found
  // target_state = ekf_prediction;

  // if (!armors_msg->armors.empty()) {
  //   // Find the closest armor with the same id
  //   Armor same_id_armor;
  //   int same_id_armors_count = 0;
  //   auto predicted_position = getArmorPositionFromState(ekf_prediction);
  //   double min_position_diff = DBL_MAX;
  //   double yaw_diff = DBL_MAX;
  //   for (const auto & armor : armors_msg->armors) {
  //     // Only consider armors with the same id
  //     if (armor.number == tracked_id) {
  //       same_id_armor = armor;
  //       same_id_armors_count++;
  //       // Calculate the difference between the predicted position and the
  //       // current armor position
  //       auto p = armor.pose.position;
  //       Eigen::Vector3d position_vec(p.x, p.y, p.z);
  //       double position_diff = (predicted_position - position_vec).norm();
  //       if (position_diff < min_position_diff) {
  //         // Find the closest armor
  //         min_position_diff = position_diff;
  //         yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
  //         tracked_armor = armor;
  //         // Update tracked armor type
  //         if (tracked_armor.type == "large" &&
  //           (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
  //         {
  //           tracked_armors_num = ArmorsNum::BALANCE_2;
  //         } else if (tracked_id == "outpost") {
  //           tracked_armors_num = ArmorsNum::OUTPOST_3;
  //         } else {
  //           tracked_armors_num = ArmorsNum::NORMAL_4;
  //         }
  //       }
  //     }
  //   }

  //   // Check if the distance and yaw difference of closest armor are within the
  //   // threshold
  //   if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
  //     // Matched armor found
  //     matched = true;
  //     auto p = tracked_armor.pose.position;
  //     // Update EKF
  //     double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
  //     measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
  //     target_state = ekf->update(measurement);
  //   } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
  //     // Matched armor not found, but there is only one armor with the same id
  //     // and yaw has jumped, take this case as the target is spinning and armor
  //     // jumped
  //     handleArmorJump(same_id_armor);
  //   } else {
  //     // No matched armor found
  //     FYT_WARN("armor_solver", "No matched armor found!");
  //   }
  // }

  // // Prevent radius from spreading
  // if (target_state(8) < 0.12) {
  //   target_state(8) = 0.12;
  //   ekf->setState(target_state);
  // } else if (target_state(8) > 0.4) {
  //   target_state(8) = 0.4;
  //   ekf->setState(target_state);
  // }

  // // Tracking state machine
  // if (tracker_state == DETECTING) {
  //   if (matched) {
  //     detect_count_++;
  //     if (detect_count_ > tracking_thres) {
  //       detect_count_ = 0;
  //       tracker_state = TRACKING;
  //       FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
  //     }
  //   } else {
  //     detect_count_ = 0;
  //     tracker_state = LOST;
  //     FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
  //   }
  // } else if (tracker_state == TRACKING) {
  //   if (!matched) {
  //     tracker_state = TEMP_LOST;
  //     lost_count_++;
  //     FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
  //   }
  // } else if (tracker_state == TEMP_LOST) {
  //   if (!matched) {
  //     lost_count_++;
  //     if (lost_count_ > lost_thres) {
  //       lost_count_ = 0;
  //       tracker_state = LOST;
  //       FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
  //     }
  //   } else {
  //     tracker_state = TRACKING;
  //     lost_count_ = 0;
  //     FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
  //   }
  // }

  // 0202 -- TJU
  // 1. EKF 预测
  // 你们的代码架构中，外部 node 已经通过 setPredictFunc 更新了 dt，这里直接预测即可
  Eigen::VectorXd ekf_prediction = ekf->predict();

  // 默认使用预测值
  bool matched = false;
  target_state = ekf_prediction; 

  if (!armors_msg->armors.empty()) {
    // 2. 寻找匹配的装甲板
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    
    for (const auto & armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;
          
          if (tracked_armor.type == "large" &&
            (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
            tracked_armors_num = ArmorsNum::BALANCE_2;
          } else if (tracked_id == "outpost") {
            tracked_armors_num = ArmorsNum::OUTPOST_3;
          } else {
            tracked_armors_num = ArmorsNum::NORMAL_4;
          }
        }
      }
    }

    // 3. 核心修改：匹配后的处理 (加入 Dynamic R 和 卡方检验)
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      matched = true;
      auto p = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

      // --- [新增 A] Dynamic R: 根据距离动态调整测量噪声 ---
      double dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      // 距离每增加，噪声线性放大，解决远距离抖动
      double r_scale = (dist > 1.0) ? dist : 1.0; 
      
      ekf->R.setIdentity();
      ekf->R(0, 0) = ekf->R(1, 1) = 4e-4 * r_scale; // XY 噪声
      ekf->R(2, 2) = 9e-4 * r_scale;               // Z 噪声
      ekf->R(3, 3) = 5e-3;                         // Yaw 噪声

// --- [新增 B] Chi-square Test: 卡方检验 ---
      bool is_outlier = false;
      double nis = 0.0; // 【关键修正】：在 if 外面提前声明变量

      if (tracker_state == TRACKING) {
          // 构造雅可比矩阵 H
          Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 10);
          double yaw_pred = ekf_prediction(6);
          double r_pred = ekf_prediction(8);
          double sa = sin(yaw_pred);
          double ca = cos(yaw_pred);

          H(0, 0) = 1; H(0, 6) = r_pred * sa; H(0, 8) = -ca;
          H(1, 2) = 1; H(1, 6) = -r_pred * ca; H(1, 8) = -sa;
          H(2, 4) = 1; H(2, 9) = 1;
          H(3, 6) = 1;

          // 计算残差
          Eigen::Vector4d z_hat;
          z_hat(0) = ekf_prediction(0) - r_pred * ca;
          z_hat(1) = ekf_prediction(2) - r_pred * sa;
          z_hat(2) = ekf_prediction(4) + ekf_prediction(9);
          z_hat(3) = yaw_pred;

          Eigen::Vector4d y = measurement - z_hat;
          while (y(3) > M_PI) y(3) -= 2 * M_PI;
          while (y(3) < -M_PI) y(3) += 2 * M_PI;

          // 计算马氏距离
          Eigen::MatrixXd S = H * ekf->P_pri * H.transpose() + ekf->R;
          
          // 【修正】：这里直接赋值，不要写 double nis = ...
          nis = y.transpose() * S.inverse() * y;

          // 阈值判断
          if (nis > 20.0) {
              is_outlier = true;
              // 这里不需要 warn 了，统一在下面 warn
          }
      }

      // --- [新增 C] 只有通过检验才更新 ---
      if (!is_outlier) {
          target_state = ekf->update(measurement);
      } else {
          FYT_WARN("armor_solver", "Violent motion detected (NIS=%.2f)! Resetting EKF...", nis);
          
          // 强制重置 EKF 状态为当前观测值
          initEKF(tracked_armor);
      }

    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      handleArmorJump(same_id_armor);
    } else {
      FYT_WARN("armor_solver", "No matched armor found!");
    }
  }

  // 4. 状态限制与状态机 (保持原样)
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf->setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf->setState(target_state);
  }

  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }

}

void Tracker::initEKF(const Armor & a) noexcept
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(X_N);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
}

// 0203 TJU
// void Tracker::handleArmorJump(const Armor & current_armor) noexcept
// {
//   double last_yaw = target_state(6);
//   double yaw = orientationToYaw(current_armor.pose.orientation);

//   if (abs(yaw - last_yaw) > 0.4) {
//     // Armor angle also jumped, take this case as target spinning
//     target_state(6) = yaw;
//     // Only 4 armors has 2 radius and height
//     if (tracked_armors_num == ArmorsNum::NORMAL_4) {
//       d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
//       std::swap(target_state(8), another_r);
//       d_zc = d_zc == 0 ? -d_za : 0;
//       target_state(9) = d_zc;
//     }
//     FYT_DEBUG("armor_solver", "Armor Jump!");
//   }

//   auto p = current_armor.pose.position;
//   Eigen::Vector3d current_p(p.x, p.y, p.z);
//   Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

//   if ((current_p - infer_p).norm() > max_match_distance_) {
//     // If the distance between the current armor and the inferred armor is too
//     // large, the state is wrong, reset center position and velocity in the
//     // state
//     d_zc = 0;
//     double r = target_state(8);
//     target_state(0) = p.x + r * cos(yaw);  // xc
//     target_state(1) = 0;                   // vxc
//     target_state(2) = p.y + r * sin(yaw);  // yc
//     target_state(3) = 0;                   // vyc
//     target_state(4) = p.z;                 // zc
//     target_state(5) = 0;                   // vzc
//     target_state(9) = d_zc;                // d_zc
//     FYT_WARN("armor_solver", "State wrong!");
//   }

//   ekf->setState(target_state);
// }

void Tracker::handleArmorJump(const Armor & current_armor) noexcept
{
  double last_yaw = target_state(6);
  double yaw = orientationToYaw(current_armor.pose.orientation);

  // 1. 处理角度跳变 (这是核心，必须保留)
  if (abs(yaw - last_yaw) > 0.4) {
    target_state(6) = yaw;
    // 处理大小板/平衡步兵的半径切换
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    FYT_DEBUG("armor_solver", "Armor Jump! Yaw diff: %.2f", abs(yaw - last_yaw));
  }

  // 2. 检查位置一致性 (防止 EKF 发散)
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  // 【优化点】：适当放宽这里的判定阈值
  // 之前的 max_match_distance_ 可能太紧，导致旋转时容易判定为 State Wrong 而重置速度
  if ((current_p - infer_p).norm() > max_match_distance_ * 1.5) { // 稍微放宽一点，比如乘 1.5
    
    // 如果位置偏差真的太大，说明 EKF 彻底废了，只能重置
    // 注意：这一步会把速度 (vxc, vyc) 重置为 0，这会导致移动时的预测瞬间丢失
    // 这就是为什么平地误触会导致“卡顿”
    
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc (速度清零)
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc (速度清零)
    target_state(4) = p.z;                 // zc
    target_state(5) = 0;                   // vzc
    target_state(9) = d_zc;                // d_zc
    FYT_WARN("armor_solver", "State wrong! Resetting velocity.");
  }

  ekf->setState(target_state);
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q) noexcept
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x) noexcept
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace fyt::auto_aim
