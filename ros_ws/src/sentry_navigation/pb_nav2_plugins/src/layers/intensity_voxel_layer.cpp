// Copyright 2025 Lihan Chen
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

#include "pb_nav2_plugins/layers/intensity_voxel_layer.hpp"

#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#define VOXEL_BITS 16

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;

namespace pb_nav2_costmap_2d
{

void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ =
    combination_method_from_int(node->get_parameter(name_ + ".combination_method").as_int());

  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  
  // Footprint-based filtering parameters
  use_footprint_filter_ = node->declare_parameter(name_ + ".use_footprint_filter", false);
  footprint_padding_ = node->declare_parameter(name_ + ".footprint_padding", 0.0);
  
  // Initialize padded footprint
  if (use_footprint_filter_) {
    auto footprint = getFootprint();
    for (const auto& point : footprint) {
      geometry_msgs::msg::Point p;
      // Add padding to footprint (expand outward)
      double angle = std::atan2(point.y, point.x);
      double dist = std::sqrt(point.x * point.x + point.y * point.y);
      p.x = (dist + footprint_padding_) * std::cos(angle);
      p.y = (dist + footprint_padding_) * std::sin(angle);
      p.z = 0.0;
      padded_footprint_.push_back(p);
    }
  }
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() {}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  // update origin information for rolling costmap publication
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // reset maps each iteration
  resetMaps();

  // if not enabled, stop here
  if (!enabled_) {
    return;
  }

  // get the maximum sized window required to operate
  useExtraBounds(min_x, min_y, max_x, max_y);

  // get the marking observations
  bool current = true;
  std::vector<Observation> observations, clearing_observations;
  current = getMarkingObservations(observations) && current;
  current = getClearingObservations(clearing_observations) && current;

  // update the global current status
  current_ = current;

  // raytrace freespace: clear costmap cells along the line of sight from sensor origin to each
  // point, so that when a dynamic obstacle moves away the stale cells are cleared.
  // This delegates to ObstacleLayer::raytraceFreespace which handles the 2-D Bresenham logic.
  for (const auto & obs : clearing_observations) {
    raytraceFreespace(obs, min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (const auto & obs : observations) {
    double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      double px = *it_x, py = *it_y, pz = *it_z;

      // if the obstacle is too low/high, we won't add it
      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
        continue;
      }

      // if the intensity is not in the range we want, we won't add it
      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far/close enough away... we won't consider it
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) {
        continue;
      }
      
      // Footprint-based filtering: check if point is inside robot footprint
      if (use_footprint_filter_ && !padded_footprint_.empty()) {
        double rel_x = px - robot_x;
        double rel_y = py - robot_y;
        
        // Rotate point to robot's local frame
        double cos_yaw = std::cos(-robot_yaw);
        double sin_yaw = std::sin(-robot_yaw);
        double local_x = rel_x * cos_yaw - rel_y * sin_yaw;
        double local_y = rel_x * sin_yaw + rel_y * cos_yaw;
        
        if (isPointInPolygon(local_x, local_y, padded_footprint_)) {
          continue;  // Skip points inside robot footprint
        }
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch(static_cast<double>(px), static_cast<double>(py), min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

// Ray casting algorithm to check if a point is inside a polygon
bool IntensityVoxelLayer::isPointInPolygon(
  double x, double y, const std::vector<geometry_msgs::msg::Point>& polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  
  bool inside = false;
  size_t j = polygon.size() - 1;
  
  for (size_t i = 0; i < polygon.size(); i++) {
    if (((polygon[i].y > y) != (polygon[j].y > y)) &&
        (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) /
             (polygon[j].y - polygon[i].y) + polygon[i].x))
    {
      inside = !inside;
    }
    j = i;
  }
  
  return inside;
}

}  // namespace pb_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::IntensityVoxelLayer, nav2_costmap_2d::Layer)
