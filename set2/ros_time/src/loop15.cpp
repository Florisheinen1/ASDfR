#include "ros_time/Loop15.h"

Loop15::Loop15() : Node("loop15")
{
    // iniializing QoS parameters
    this->declare_parameter<std::string>("reliability", "reliable");
    this->declare_parameter<std::string>("history", "keep_last");
    this->declare_parameter<std::string>("durability", "volatile");
    this->declare_parameter<int>("depth", 10);

    // Reading the QoS parameters
    std::string reliability = this->get_parameter("reliability").as_string();
    std::string history = this->get_parameter("history").as_string();
    std::string durability = this->get_parameter("durability").as_string();
    int depth = this->get_parameter("depth").as_int();

    // Printing the QoS settings
    RCLCPP_INFO(this->get_logger(), "QoS: %s, %s, %s, depth=%d",
                reliability.c_str(), history.c_str(), durability.c_str(), depth);

    // Inputing the QoS parameters
    rclcpp::QoS qos(depth);
    if (history == "keep_all")
        qos.keep_all();
    else
        qos.keep_last(depth); // defining the QoS using the library using if else by comparing the input

    if (reliability == "best_effort")
        qos.best_effort();
    else
        qos.reliable();

    if (durability == "transient_local")
        qos.transient_local();
    else
        qos.durability_volatile();

    // Creating subscriber and publisher by including the QoS
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "seq15_topic", qos, std::bind(&Loop15::callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("loop15_topic", qos);

    // Output that node is created
    RCLCPP_INFO(this->get_logger(), "Loop15 node started.");
}

// Called when message is received from subscriber (seq15_topic)
void Loop15::callback(const std_msgs::msg::String::SharedPtr msg)
{
    publisher_->publish(*msg); // Publish the same message back
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Loop15>());
    rclcpp::shutdown();
    return 0;
}
