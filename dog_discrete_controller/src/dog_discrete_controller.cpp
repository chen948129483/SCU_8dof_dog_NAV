#include "dog_discrete_controller/dog_discrete_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include <cmath>
#include "usart_pkg/msg/action.hpp"


using nav2_util::declare_parameter_if_not_declared;

namespace dog_nav
{

// =========================================================================
// 1. 生命周期：配置阶段 (读取参数，初始化接口)
// =========================================================================
void DogDiscreteController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)//特意取消注释
{
  node_ = parent;
  auto node = node_.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // 声明并读取参数 (在 nav2_params.yaml 中配置)
  declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.08)); // 约 4.5 度
  declare_parameter_if_not_declared(node, plugin_name_ + ".goal_stop_dist", rclcpp::ParameterValue(0.22));

  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".yaw_tolerance", yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".goal_stop_dist", goal_stop_dist_);

  // 订阅狗的执行状态 (必须与底层发布的话题名完全一致)
  status_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/dog_status/is_stepping", 10,
    std::bind(&DogDiscreteController::statusCallback, this, std::placeholders::_1));

  // 发布离散指令 (第一个字节发送动作指令1:前进, 2:后退, 3:左转, 4:右转, 0:停止，第二个字节发送速度指令0：慢，1：中，2：会)
  cmd_pub_ = node->create_publisher<usart_pkg::msg::Action>("/dog_step_cmd", 10);

  collision_checker_ = std::make_shared<DogCollisionChecker>(node, costmap_ros_);
}

// =========================================================================
// 2. 生命周期：激活、暂停、清理
// =========================================================================
void DogDiscreteController::cleanup()
{
  cmd_pub_.reset();
  status_sub_.reset();
}

void DogDiscreteController::activate()
{
  cmd_pub_->on_activate();
}

void DogDiscreteController::deactivate()
{
  cmd_pub_->on_deactivate();
}

// =========================================================================
// 3. 数据接收与回调
// =========================================================================
void DogDiscreteController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(logger_, "setPlan received: %zu poses", global_plan_.poses.size());
}

void DogDiscreteController::statusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_stepping_ = msg->data;
  status_tick_received_ = true;
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "statusCallback: is_stepping=%s, command tick received",
      is_stepping_ ? "true" : "false");
  }
}

void DogDiscreteController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // 离散步态不需要连续动态限速，保持为空即可
}

// =========================================================================
// 4. 辅助函数：在全局路径上寻找预瞄点 (Lookahead Point)
// =========================================================================
geometry_msgs::msg::PoseStamped DogDiscreteController::getLookAheadPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & transformed_plan)
{ 
  // 如果路径为空，保底返回当前位置
  if (transformed_plan.poses.empty()) {
    return current_pose;
  }

  // 因为 transformed_plan 已经被 transformGlobalPlan 裁剪过了
  // 它的第 0 个点就是离狗最近的点，我们直接从这里往后找
  for (const auto & pose : transformed_plan.poses) {
    double dx = pose.pose.position.x - current_pose.pose.position.x;
    double dy = pose.pose.position.y - current_pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist >= lookahead_dist_) {
      return pose; // 找到了第一个超过预瞄距离的点
    }
  }

  // 保底：如果整条路径都很短，没超过预瞄距离，就返回路径的最后一个点
  return transformed_plan.poses.back();
}

// =========================================================================
// 5. 🌟 核心算法：路径追踪与离散指令下发 (10Hz调用)
// =========================================================================
geometry_msgs::msg::TwistStamped DogDiscreteController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // 构造返回给 Nav2 框架的零速度
  geometry_msgs::msg::TwistStamped zero_cmd_vel;
  zero_cmd_vel.header.frame_id = "base_link";
  zero_cmd_vel.twist.linear.x = 0.0;
  zero_cmd_vel.twist.angular.z = 0.0;

  // 核心约束：状态话题作为节拍信号。收到任意 true/false 后，只允许下发一条指令。
  if (!status_tick_received_) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 1000,
        "Skip publish: waiting for /dog_status/is_stepping tick.");
    }
    return zero_cmd_vel;
  }

  // A. 终点检查：离散步态到终点附近就直接站立，不继续追最后一点的朝向。
  if (global_plan_.poses.empty()) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 1000,
        "Skip publish: global_plan_ is empty (setPlan not received or empty path).");
    }
    return zero_cmd_vel;
  }

  // 1. 局部路径转换与裁剪逻辑 (保持你原来的代码逻辑)
  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = pose.header.frame_id; 

  try {
    for (auto global_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      global_pose.header.frame_id = global_plan_.header.frame_id;
      global_pose.header.stamp = rclcpp::Time(0); 
      transformed_pose = tf_->transform(global_pose, pose.header.frame_id);
      local_plan.poses.push_back(transformed_pose);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000, "TF转换失败！");
    
    // 安全起见，发送站立/停止指令 (1)
    usart_pkg::msg::Action stop_cmd;
    stop_cmd.action_id = 1; // standup
    stop_cmd.speed_level = 0;
    cmd_pub_->publish(stop_cmd);
    status_tick_received_ = false;
    return zero_cmd_vel;
  }

  // 裁剪路径
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_index = 0;
  for (size_t i = 0; i < local_plan.poses.size(); ++i) {
    double dx = local_plan.poses[i].pose.position.x - pose.pose.position.x;
    double dy = local_plan.poses[i].pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
  }
  local_plan.poses.erase(local_plan.poses.begin(), local_plan.poses.begin() + closest_index);

  const auto & goal_pose_local = local_plan.poses.back();
  const double goal_dx = goal_pose_local.pose.position.x - pose.pose.position.x;
  const double goal_dy = goal_pose_local.pose.position.y - pose.pose.position.y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);

  if (goal_distance <= goal_stop_dist_) {
    RCLCPP_INFO(
      logger_,
      "已进入终点站立半径 %.3f m (当前距离 %.3f m)，切换至站立状态。",
      goal_stop_dist_,
      goal_distance);
    usart_pkg::msg::Action stand_cmd;
    stand_cmd.action_id = 1; // standup
    stand_cmd.speed_level = 0;
    cmd_pub_->publish(stand_cmd);
    last_motion_action_ = 1;
    status_tick_received_ = false;
    return zero_cmd_vel;
  }

  // 2. 获取预瞄点
  geometry_msgs::msg::PoseStamped target_pose = getLookAheadPoint(pose, local_plan);

  // 3. 计算偏差
  double dx = target_pose.pose.position.x - pose.pose.position.x;
  double dy = target_pose.pose.position.y - pose.pose.position.y;
  double distance_error = std::hypot(dx, dy);

  geometry_msgs::msg::Pose goal_pose = global_plan_.poses.back().pose;
  if (goal_checker->isGoalReached(pose.pose, goal_pose, velocity)) {
    RCLCPP_INFO(logger_, "已到达终点，切换至站立状态。");
    usart_pkg::msg::Action stand_cmd;
    stand_cmd.action_id = 1; // standup
    stand_cmd.speed_level = 0;
    cmd_pub_->publish(stand_cmd);
    status_tick_received_ = false;
    return zero_cmd_vel;
  }

  // B. 角度计算
  double target_yaw = std::atan2(dy, dx);
  double current_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw_error = angles::shortest_angular_distance(current_yaw, target_yaw);

  // 4. 【新版离散决策树】
  usart_pkg::msg::Action step_cmd;

  // 4.1 先在脑子里算出理论上的最优动作意图 (Intended Action)
  uint8_t intended_action = 1; // 默认站立
  if (std::abs(yaw_error) > 2.6) {
    intended_action = 9; // 想后退
  } else if (std::abs(yaw_error) > yaw_tolerance_) {
    intended_action = (yaw_error > 0) ? 4 : 5; // 想左转或右转
  } else if (distance_error > 0.04) {
    intended_action = 2; // 想前进
  }
  // 4.2 【测试期强制安全锁】：映射出对应的物理步长参数
  // 无论多远，目前全部锁死为 0 档极慢速测试
  uint8_t intended_speed = 0;  // 永远下发 0 档速度
  double physical_step = 0.09; // 0档对应的平移步长 (9cm)
  double turn_angle = 0.1;     // 0档对应的旋转弧度 (约5.7度)
  step_cmd.speed_level = 0; // 默认所有动作速度等级均为 0

  // 4.3 拿着算好的意图和步长参数，去查 70x45cm 的真实轮廓会不会撞墙
// 4.3 拿着算好的意图和步长参数，去查 70x45cm 的真实轮廓会不会撞墙
auto getReverseAction = [](uint8_t action) -> uint8_t {
  switch (action) {
    case 2: return 9; // 上一步前进，这次后退
    case 9: return 2; // 上一步后退，这次前进
    case 4: return 5; // 上一步左转，这次右转
    case 5: return 4; // 上一步右转，这次左转
    default: return 1; // 站立或未知动作，保底站立
  }
};

if (collision_checker_->isCollisionFree(pose, intended_action, physical_step, turn_angle)) {
  // 正常执行原本规划出来的动作
  step_cmd.action_id = intended_action;
  step_cmd.speed_level = intended_speed;

  // 只记录真正的运动动作，不记录站立
  if (intended_action != 1) {
    last_motion_action_ = intended_action;
  }

} else {
  // 发生碰撞预警，不直接站立，而是尝试反向补偿动作
  uint8_t recovery_action = getReverseAction(last_motion_action_);

  RCLCPP_WARN(
    logger_,
    "防撞触发！原动作 %u 被拦截，尝试根据上一步动作 %u 执行反向恢复动作 %u",
    intended_action,
    last_motion_action_,
    recovery_action);

  if (recovery_action != 1 &&
      collision_checker_->isCollisionFree(pose, recovery_action, physical_step, turn_angle)) {

    step_cmd.action_id = recovery_action;
    step_cmd.speed_level = intended_speed;

    // 反向动作也算一次真实运动，更新上一步动作
    last_motion_action_ = recovery_action;

    RCLCPP_WARN(
      logger_,
      "反向恢复动作通过防撞检测，执行 action_id=%u",
      recovery_action);

  } else {
    // 如果反向动作也不安全，再做最终保底
    RCLCPP_WARN(
      logger_,
      "反向恢复动作 %u 也存在碰撞风险，最终切换至站立保护。",
      recovery_action);

    step_cmd.action_id = 1; // standup / 站立
    step_cmd.speed_level = 0;
  }
}
  // 5. 下发指令
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "Publish /dog_step_cmd: action_id=%u speed_level=%u", step_cmd.action_id, step_cmd.speed_level);
  }
  cmd_pub_->publish(step_cmd);
  status_tick_received_ = false;

  return zero_cmd_vel;
}

}// namespace dog_nav

// 注册插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_nav::DogDiscreteController, nav2_core::Controller)
