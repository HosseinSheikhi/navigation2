#ifndef CEILING_LAYER_HPP_
#define CEILING_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "algorithm"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "custom_roi_srv/srv/roi.hpp"
#include "memory.h"
#include <algorithm>

namespace nav2_ceiling_perception_plugin
{

class CeilingLayer : public nav2_costmap_2d::CostmapLayer
{
 public:
  CeilingLayer();

  virtual void onInitialize();
  virtual void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double * min_x,
      double * min_y,
      double * max_x,
      double * max_y);
  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();
  virtual bool isClearable() {return false;}

 private:

  double roi_min_x_{0}, roi_min_y_{0}, roi_max_x_{0}, roi_max_y_{0}; ///< ceiling_perception map ROI
  unsigned int ceiling_size_x_, ceiling_size_y_; ///< desired size of x and y in meter based on ceiling_perception map coverage
  double ceiling_origin_x_, ceiling_origin_y_; ///< (x,y) origin of ceiling_perception map
  double ceiling_resolution_; ///< resolution of ceiling_perception map, must be as same as SLAM's map in current implementation

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ceiling_map_sub_; ///< subscribe to ceiling perception map
  void ceiling_map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr map);///< callback for ceiling_perception map
  std::vector<signed char> ceiling_map_; ///< keeps a copy of ceiling_perception map


  rclcpp::Service<custom_roi_srv::srv::ROI>::SharedPtr ceiling_ROI_srv_; ///< gives service to static_layer to share the desired changes (costmap's size, origin, etc.)
  void handle_ceiling_roi_service(std::shared_ptr<custom_roi_srv::srv::ROI::Request> request,
                                  std::shared_ptr<custom_roi_srv::srv::ROI::Response> response);


  unsigned int counter = 0; ///< a temporary variable

  //parameters
  void getParameters(); ///< defines and reads parameters

};

}  // namespace nav2_ceiling_perception_plugin

#endif  // CEILING_LAYER_HPP_
