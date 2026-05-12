#include <rclcpp/rclcpp.hpp>

class TopicRelay : public rclcpp::Node
{
public:
  TopicRelay() : Node("topic_relay")
  {
    declare_parameter("input", std::string{});
    declare_parameter("output", std::string{});
    declare_parameter("type", std::string{});

    const auto input = get_parameter("input").as_string();
    const auto output = get_parameter("output").as_string();
    const auto type   = get_parameter("type").as_string();

    if (input.empty() || output.empty() || type.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameters 'input', 'output', and 'type' are all required.");
      throw std::runtime_error("Missing required parameters");
    }

    pub_ = create_generic_publisher(output, type, 10);
    sub_ = create_generic_subscription(
      input, type, 10,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        pub_->publish(*msg);
      });

    RCLCPP_INFO(get_logger(), "Relaying  %s  ->  %s  [%s]",
      input.c_str(), output.c_str(), type.c_str());
  }

private:
  rclcpp::GenericPublisher::SharedPtr    pub_;
  rclcpp::GenericSubscription::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicRelay>());
  rclcpp::shutdown();
  return 0;
}
