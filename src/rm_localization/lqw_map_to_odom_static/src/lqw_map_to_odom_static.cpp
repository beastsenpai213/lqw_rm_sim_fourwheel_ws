#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher()
  : Node("lqw_static_turtle_tf2_broadcaster")
  {
    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
  }
private:
  void make_transforms()
  {
    rclcpp::Time now;
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = atof("0");
    t.transform.translation.y = atof("0");
    t.transform.translation.z = atof("0");
    tf2::Quaternion q;
    q.setRPY(
      atof("0"),
      atof("0"),
      atof("0"));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_publisher_->sendTransform(t);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}