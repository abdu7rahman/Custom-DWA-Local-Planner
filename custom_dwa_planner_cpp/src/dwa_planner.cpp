#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <random>

using namespace std;

class DWALocalPlanner : public rclcpp::Node {
public:
    DWALocalPlanner() : Node("dwa_planner") {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&DWALocalPlanner::odom_callback, this, std::placeholders::_1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DWALocalPlanner::scan_callback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = create_publisher<visualization_msgs::msg::Marker>("/visual_paths", 10);

        max_speed_ = 0.15;
        max_turn_ = 2.5;
        step_time_ = 0.1;

        ask_for_goal();
        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(step_time_ * 1000)), std::bind(&DWALocalPlanner::movement_loop, this));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
    double goal_x_, goal_y_;
    bool goal_reached_ = false;
    double max_speed_, max_turn_, step_time_;
    random_device rd_;
    mt19937 gen_{rd_()};
    uniform_real_distribution<double> speed_dist_{0, max_speed_};
    uniform_real_distribution<double> turn_dist_{-max_turn_, max_turn_};

    void ask_for_goal() {
        cout << "Enter goal X: ";
        cin >> goal_x_;
        cout << "Enter goal Y: ";
        cin >> goal_y_;
        goal_reached_ = false;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_data_ = msg;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_data_ = msg;
    }

    vector<pair<double, double>> predict_motion(double speed, double turn_rate) {
        vector<pair<double, double>> path;
        if (!odom_data_) return path;

        double x = odom_data_->pose.pose.position.x;
        double y = odom_data_->pose.pose.position.y;
        tf2::Quaternion q(odom_data_->pose.pose.orientation.x, odom_data_->pose.pose.orientation.y,
                          odom_data_->pose.pose.orientation.z, odom_data_->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        for (int i = 0; i < 100; ++i) {
            yaw += turn_rate * step_time_;
            x += speed * cos(yaw) * step_time_;
            y += speed * sin(yaw) * step_time_;
            path.emplace_back(x, y);
        }
        return path;
    }

    pair<double, double> generate_path() {
        double speed = speed_dist_(gen_);
        double turn = turn_dist_(gen_);
        vector<pair<double, double>> path = predict_motion(speed, turn);
        return {speed, turn};
    }

    double check_for_collisions(const vector<pair<double, double>>& path) {
        if (!scan_data_) return -INFINITY;
        double safety_margin = 0.3;

        for (const auto& [x, y] : path) {
            double distance = hypot(x, y);
            int scan_index = static_cast<int>((atan2(y, x) / (2 * M_PI)) * scan_data_->ranges.size());
            scan_index = max(0, min(static_cast<int>(scan_data_->ranges.size()) - 1, scan_index));

            if (distance < scan_data_->ranges[scan_index] - safety_margin) {
                return -100000;
            }
        }
        return 0;
    }
    
    pair<double, double> choose_best_path() {
        if (!odom_data_) return {0.0, 0.0};

        double current_x = odom_data_->pose.pose.position.x;
        double current_y = odom_data_->pose.pose.position.y;
        tf2::Quaternion q(odom_data_->pose.pose.orientation.x, odom_data_->pose.pose.orientation.y,
                          odom_data_->pose.pose.orientation.z, odom_data_->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (hypot(goal_x_ - current_x, goal_y_ - current_y) < 0.05) {
            if (!goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(get_logger(), "Goal reached at (%f, %f)!", goal_x_, goal_y_);
            }
            return {0.0, 0.0};
        }

        double best_score = -INFINITY;
        double best_speed = 0.05, best_turn = 0.0;

        for (int i = 0; i < 10000; ++i) {
            auto [speed, turn] = generate_path();
            vector<pair<double, double>> path = predict_motion(speed, turn);

            double goal_distance_score = -hypot(path.back().first - goal_x_, path.back().second - goal_y_) * 5;
            double angle_diff = abs(atan2(goal_y_ - current_y, goal_x_ - current_x) - yaw);
            double heading_score = -angle_diff * 2;
            double collision_risk = check_for_collisions(path);
            double smoothness_score = -0.1 * abs(turn);
            double total_score = goal_distance_score + heading_score + collision_risk + smoothness_score;

            if (total_score > best_score) {
                best_score = total_score;
                best_speed = speed;
                best_turn = turn;
            }
        }
        return {best_speed, best_turn};
    }

    void movement_loop() {
        if (goal_reached_ || !odom_data_ || !scan_data_) return;
        auto [speed, turn] = choose_best_path();
        geometry_msgs::msg::Twist move_cmd;
        move_cmd.linear.x = speed;
        move_cmd.angular.z = turn;
        cmd_pub_->publish(move_cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWALocalPlanner>());
    rclcpp::shutdown();
    return 0;
}
