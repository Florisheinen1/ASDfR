#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <unordered_map>
#include <sstream>
#include <fstream>

using namespace std::chrono;
using std::placeholders::_1;

class Seq15 : public rclcpp::Node {
public:
    Seq15() : Node("seq15"), message_id_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("seq15_topic", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "loop15_topic", 10, std::bind(&Seq15::response_callback, this, _1));

        timer_ = this->create_wall_timer(
            milliseconds(1), std::bind(&Seq15::timer_callback, this));

        // Open CSV file for writing
        csv_file_.open("timing_results.csv", std::ios::out | std::ios::trunc);
        csv_file_ << "timestamp,rtt,jitter\n";  // CSV headers

        RCLCPP_INFO(this->get_logger(), "Seq15 node started.");
    }

    ~Seq15() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void timer_callback() {
        auto timestamp = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
        std::ostringstream oss;
        oss << message_id_ << "," << timestamp;
        
        auto msg = std_msgs::msg::String();
        msg.data = oss.str();
        
        timestamps_[message_id_] = timestamp;
        message_id_++;

        RCLCPP_INFO(this->get_logger(), "Sent: %s", msg.data.c_str());
        publisher_->publish(msg);
    }

    void response_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto receive_time = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
        
        // Extract message ID and sent timestamp
        std::istringstream iss(msg->data);
        int msg_id;
        long sent_time;
        char comma;
        iss >> msg_id >> comma >> sent_time;

        // Compute Round-Trip Time (RTT)
        double round_trip_time = (receive_time - sent_time) / 1e6;  // Convert ns to ms
        
        // Compute jitter
        static double prev_rtt = 0;
        double jitter = abs(round_trip_time - prev_rtt);
        prev_rtt = round_trip_time;

        // Log results to CSV file
        if (csv_file_.is_open()) {
            csv_file_ << receive_time << "," << round_trip_time << "," << jitter << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "RTT: %.3f ms, Jitter: %.3f ms", round_trip_time, jitter);
        timestamps_.erase(msg_id);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int, long> timestamps_;
    int message_id_;
    std::ofstream csv_file_;  // CSV file handle
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Seq15>());
    rclcpp::shutdown();
    return 0;
}
