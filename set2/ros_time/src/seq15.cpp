#include "ros_time/Seq15.h"

Seq15::Seq15() : Node("seq15"), last_sent_(0), prev_rtt_(0)
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

    // Creating publisher and subscriber by including the QoS
    publisher_ = this->create_publisher<std_msgs::msg::String>("seq15_topic", qos);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "loop15_topic", qos, std::bind(&Seq15::callback, this, _1));

    // Setting a timer to run at every 1 kHz)
    timer_ = this->create_wall_timer(milliseconds(1), std::bind(&Seq15::timer_callback, this)); // 1 kHz = 1 millisecond

    // Intitializing CSV file to write the timestamp, rtt and jitter
    log_file_.open("timing_results.csv", std::ios::out | std::ios::trunc);
    log_file_ << "timestamp,rtt,jitter\n";
    // Output that node is created
    RCLCPP_INFO(this->get_logger(), "Seq15 node started.");
}

Seq15::~Seq15()
{
    if (log_file_.is_open())
        log_file_.close(); // close file
}

// sends a message with the current timestamp
void Seq15::timer_callback()
{
    auto now = steady_clock::now();
    auto timestamp = duration_cast<nanoseconds>(now.time_since_epoch()).count();

    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(timestamp); // timestamp is the unique identifier
    publisher_->publish(msg);             // publish the msg

    last_sent_ = timestamp; // store it to compare later
}

// Called when message is recived from subscriber ie loop15
void Seq15::callback(const std_msgs::msg::String::SharedPtr msg)
{
    auto now = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();

    // Extract the original timestamp from the message data
    long sent_time = std::stol(msg->data);

    // Calculate round-trip time (ms)
    double rtt = (now - sent_time) / 1e6;

    // Calculate jitter as the change in RTT from last message
    double jitter = std::abs(rtt - prev_rtt_);
    prev_rtt_ = rtt;

    // Write data in the CSV file
    if (log_file_.is_open())
    {
        log_file_ << now << "," << rtt << "," << jitter << "\n";
    }

    // output in terminal
    RCLCPP_INFO(this->get_logger(), "RTT: %.3f ms | Jitter: %.3f ms", rtt, jitter);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Seq15>());
    rclcpp::shutdown();
    return 0;
}
