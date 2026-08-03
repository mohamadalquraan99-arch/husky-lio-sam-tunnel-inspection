#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

class TunnelAnomalyDetector : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  TunnelAnomalyDetector()
  : Node("tunnel_anomaly_detector")
  {
    baseline_pcd_ = declare_parameter("baseline_pcd", std::string());
    current_pcd_ = declare_parameter("current_pcd", std::string());
    wait_for_trigger_ = declare_parameter("wait_for_trigger", false);
    trigger_topic_ = declare_parameter(
      "trigger_topic", std::string("/inspection/run_detection"));
    current_cloud_topic_ = declare_parameter(
      "current_cloud_topic", std::string("/lio_sam/mapping/map_global"));
    output_frame_ = declare_parameter("output_frame", std::string("map"));
    processing_period_s_ = declare_parameter("processing_period_s", 5.0);
    process_once_ = declare_parameter("process_once", true);
    voxel_leaf_size_m_ = declare_parameter("voxel_leaf_size_m", 0.15);
    change_distance_threshold_m_ = declare_parameter(
      "change_distance_threshold_m", 0.25);
    cluster_tolerance_m_ = declare_parameter("cluster_tolerance_m", 0.40);
    minimum_cluster_points_ = declare_parameter("minimum_cluster_points", 25);
    maximum_cluster_points_ = declare_parameter("maximum_cluster_points", 100000);
    use_icp_alignment_ = declare_parameter("use_icp_alignment", false);
    icp_max_correspondence_distance_m_ = declare_parameter(
      "icp_max_correspondence_distance_m", 1.0);
    icp_max_iterations_ = declare_parameter("icp_max_iterations", 30);
    icp_max_fitness_score_ = declare_parameter("icp_max_fitness_score", 0.20);

    if (baseline_pcd_.empty()) {
      throw std::runtime_error("Parameter 'baseline_pcd' must contain an absolute PCD path");
    }
    if (voxel_leaf_size_m_ <= 0.0 || change_distance_threshold_m_ <= 0.0) {
      throw std::runtime_error("Voxel size and change threshold must be positive");
    }

    CloudT::Ptr baseline_raw(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(baseline_pcd_, *baseline_raw) < 0) {
      throw std::runtime_error("Unable to load baseline PCD: " + baseline_pcd_);
    }

    std::vector<int> valid_indices;
    CloudT::Ptr baseline_clean(new CloudT);
    pcl::removeNaNFromPointCloud(*baseline_raw, *baseline_clean, valid_indices);
    baseline_cloud_ = voxelize(baseline_clean);
    if (baseline_cloud_->empty()) {
      throw std::runtime_error("Baseline PCD contains no usable points");
    }

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.reliable().transient_local();
    anomaly_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inspection/anomaly_points", output_qos);
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inspection/aligned_current_cloud", output_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/inspection/anomaly_markers", output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/inspection/anomaly_status", output_qos);

    last_processing_time_ = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(processing_period_s_));

    if (wait_for_trigger_) {
      trigger_sub_ = create_subscription<std_msgs::msg::String>(
        trigger_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&TunnelAnomalyDetector::trigger_callback, this, std::placeholders::_1));
    } else if (current_pcd_.empty()) {
      current_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        current_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TunnelAnomalyDetector::cloud_callback, this, std::placeholders::_1));
    } else {
      current_pcd_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TunnelAnomalyDetector::load_current_pcd_once, this));
    }

    RCLCPP_INFO(
      get_logger(), "Loaded baseline '%s': %zu filtered points",
      baseline_pcd_.c_str(), baseline_cloud_->size());
    if (wait_for_trigger_) {
      RCLCPP_INFO(
        get_logger(), "Waiting for saved-map trigger on %s",
        trigger_topic_.c_str());
    } else if (current_pcd_.empty()) {
      RCLCPP_INFO(
        get_logger(), "Listening for current map on %s",
        current_cloud_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Dense offline comparison requested with '%s'",
        current_pcd_.c_str());
    }
  }

private:
  CloudT::Ptr voxelize(const CloudT::ConstPtr & input) const
  {
    CloudT::Ptr output(new CloudT);
    pcl::VoxelGrid<PointT> filter;
    const float leaf = static_cast<float>(voxel_leaf_size_m_);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(input);
    filter.filter(*output);
    return output;
  }

  void trigger_callback(const std_msgs::msg::String::SharedPtr message)
  {
    if (message->data.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring anomaly trigger with an empty PCD path");
      return;
    }

    current_pcd_ = message->data;
    processed_once_ = false;
    RCLCPP_INFO(
      get_logger(), "Mission manager requested comparison with '%s'",
      current_pcd_.c_str());
    load_current_pcd_once();
  }

  void load_current_pcd_once()
  {
    CloudT::Ptr current(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(current_pcd_, *current) < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to load current PCD: %s", current_pcd_.c_str());
      if (current_pcd_timer_) {
        current_pcd_timer_->cancel();
      }
      return;
    }

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*current, message);
    message.header.frame_id = output_frame_;
    message.header.stamp = now();
    cloud_callback(std::make_shared<sensor_msgs::msg::PointCloud2>(message));
    if (current_pcd_timer_) {
      current_pcd_timer_->cancel();
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (process_once_ && processed_once_) {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(
      now_steady - last_processing_time_).count();
    if (elapsed < processing_period_s_) {
      return;
    }
    last_processing_time_ = now_steady;

    CloudT::Ptr current_raw(new CloudT);
    pcl::fromROSMsg(*message, *current_raw);
    std::vector<int> valid_indices;
    CloudT::Ptr current_clean(new CloudT);
    pcl::removeNaNFromPointCloud(*current_raw, *current_clean, valid_indices);
    if (current_clean->empty()) {
      RCLCPP_WARN(get_logger(), "Received an empty current point cloud");
      return;
    }

    CloudT::Ptr current_filtered = voxelize(current_clean);
    CloudT::Ptr aligned(new CloudT(*current_filtered));

    if (use_icp_alignment_) {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setInputSource(current_filtered);
      icp.setInputTarget(baseline_cloud_);
      icp.setMaximumIterations(icp_max_iterations_);
      icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_m_);
      icp.align(*aligned);

      if (!icp.hasConverged() || icp.getFitnessScore() > icp_max_fitness_score_) {
        RCLCPP_WARN(
          get_logger(),
          "ICP rejected (converged=%s, fitness=%.4f); using the existing map frame",
          icp.hasConverged() ? "true" : "false", icp.getFitnessScore());
        *aligned = *current_filtered;
      } else {
        RCLCPP_INFO(
          get_logger(), "ICP alignment accepted with fitness %.4f",
          icp.getFitnessScore());
      }
    }

    publish_cloud(aligned, aligned_cloud_pub_, message->header.stamp, message->header.frame_id);

    pcl::KdTreeFLANN<PointT> baseline_tree;
    baseline_tree.setInputCloud(baseline_cloud_);
    CloudT::Ptr candidates(new CloudT);
    const float threshold_squared = static_cast<float>(
      change_distance_threshold_m_ * change_distance_threshold_m_);
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_distance_squared(1);

    for (const auto & point : aligned->points) {
      if (baseline_tree.nearestKSearch(
          point, 1, nearest_index, nearest_distance_squared) == 0 ||
        nearest_distance_squared.front() > threshold_squared)
      {
        candidates->push_back(point);
      }
    }
    candidates->width = static_cast<std::uint32_t>(candidates->size());
    candidates->height = 1;
    candidates->is_dense = true;

    std::vector<pcl::PointIndices> cluster_indices;
    if (!candidates->empty()) {
      pcl::search::KdTree<PointT>::Ptr candidate_tree(new pcl::search::KdTree<PointT>);
      candidate_tree->setInputCloud(candidates);
      pcl::EuclideanClusterExtraction<PointT> clustering;
      clustering.setClusterTolerance(cluster_tolerance_m_);
      clustering.setMinClusterSize(minimum_cluster_points_);
      clustering.setMaxClusterSize(maximum_cluster_points_);
      clustering.setSearchMethod(candidate_tree);
      clustering.setInputCloud(candidates);
      clustering.extract(cluster_indices);
    }

    CloudT::Ptr confirmed(new CloudT);
    std::vector<CloudT::Ptr> clusters;
    clusters.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      CloudT::Ptr cluster(new CloudT);
      cluster->reserve(indices.indices.size());
      for (const int index : indices.indices) {
        cluster->push_back((*candidates)[static_cast<std::size_t>(index)]);
      }
      cluster->width = static_cast<std::uint32_t>(cluster->size());
      cluster->height = 1;
      cluster->is_dense = true;
      *confirmed += *cluster;
      clusters.push_back(cluster);
    }

    publish_cloud(confirmed, anomaly_cloud_pub_, message->header.stamp, message->header.frame_id);
    publish_markers(clusters, message->header.stamp, message->header.frame_id);
    publish_status(clusters, candidates->size(), aligned->size());
    processed_once_ = true;
  }

  void publish_cloud(
    const CloudT::ConstPtr & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & input_frame)
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_.empty() ? input_frame : output_frame_;
    publisher->publish(output);
  }

  void publish_markers(
    const std::vector<CloudT::Ptr> & clusters,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & input_frame)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    const std::string frame = output_frame_.empty() ? input_frame : output_frame_;
    int marker_id = 0;
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      PointT minimum;
      PointT maximum;
      pcl::getMinMax3D(*clusters[index], minimum, maximum);

      visualization_msgs::msg::Marker box;
      box.header.frame_id = frame;
      box.header.stamp = stamp;
      box.ns = "structural_change_boxes";
      box.id = marker_id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = 0.5 * (minimum.x + maximum.x);
      box.pose.position.y = 0.5 * (minimum.y + maximum.y);
      box.pose.position.z = 0.5 * (minimum.z + maximum.z);
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(0.10f, maximum.x - minimum.x);
      box.scale.y = std::max(0.10f, maximum.y - minimum.y);
      box.scale.z = std::max(0.10f, maximum.z - minimum.z);
      box.color.r = 1.0f;
      box.color.g = 0.05f;
      box.color.b = 0.05f;
      box.color.a = 0.35f;
      array.markers.push_back(box);

      visualization_msgs::msg::Marker label;
      label.header = box.header;
      label.ns = "structural_change_labels";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = box.pose.position.x;
      label.pose.position.y = box.pose.position.y;
      label.pose.position.z = maximum.z + 0.40;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = "Possible structural change " + std::to_string(index + 1);
      array.markers.push_back(label);
    }

    marker_pub_->publish(array);
  }

  void publish_status(
    const std::vector<CloudT::Ptr> & clusters,
    std::size_t candidate_points,
    std::size_t compared_points)
  {
    std_msgs::msg::String status;
    std::ostringstream stream;
    if (clusters.empty()) {
      stream << "No meaningful structural change detected";
      RCLCPP_INFO(
        get_logger(), "%s (%zu candidates from %zu compared points)",
        stream.str().c_str(), candidate_points, compared_points);
    } else {
      stream << "Possible structural change detected: " << clusters.size()
             << " cluster(s)";
      RCLCPP_WARN(
        get_logger(), "%s (%zu candidates from %zu compared points)",
        stream.str().c_str(), candidate_points, compared_points);
    }
    status.data = stream.str();
    status_pub_->publish(status);
  }

  std::string baseline_pcd_;
  std::string current_pcd_;
  bool wait_for_trigger_{false};
  std::string trigger_topic_;
  std::string current_cloud_topic_;
  std::string output_frame_;
  double processing_period_s_{5.0};
  bool process_once_{true};
  bool processed_once_{false};
  double voxel_leaf_size_m_{0.15};
  double change_distance_threshold_m_{0.25};
  double cluster_tolerance_m_{0.40};
  int minimum_cluster_points_{25};
  int maximum_cluster_points_{100000};
  bool use_icp_alignment_{false};
  double icp_max_correspondence_distance_m_{1.0};
  int icp_max_iterations_{30};
  double icp_max_fitness_score_{0.20};

  CloudT::Ptr baseline_cloud_;
  std::chrono::steady_clock::time_point last_processing_time_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr current_pcd_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr anomaly_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TunnelAnomalyDetector>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("tunnel_anomaly_detector"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
