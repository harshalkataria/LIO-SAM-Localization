#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

class CovarianceEstimator {
   public:
    CovarianceEstimator() {
        initializeKalmanFilter();
        // Subscribe to odometry topics
        lio_odom_sub = nh.subscribe("/lio_sam/mapping/odometry", 5, &CovarianceEstimator::lioOdomCallback, this);
        gazebo_odom_sub = nh.subscribe("/gem/base_footprint/odom", 5, &CovarianceEstimator::gazeboOdomCallback, this);
        // initialize Kalman filter
    }

    void run() { ros::spin(); }

   private:
    ros::NodeHandle nh;                                // Node handle
    ros::Subscriber lio_odom_sub;                      // Subscriber for LIO-SAM odometry
    ros::Subscriber gazebo_odom_sub;                   // Subscriber for Gazebo Ground Truth odometry
    Eigen::VectorXd state;                             // State vector [x, y, z, roll, pitch, yaw]
    Eigen::MatrixXd covariance;                        // Covariance matrix
    Eigen::MatrixXd F;                                 // State transition model
    Eigen::MatrixXd H;                                 // Measurement model
    Eigen::MatrixXd R;                                 // Measurement noise covariance
    Eigen::MatrixXd Q;                                 // Process noise covariance
    nav_msgs::Odometry lio_odom;                       // LIO-SAM odometry
    nav_msgs::Odometry gazebo_odom;                    // Gazebo Ground Truth odometry
    bool lio_odom_received = false;                    // Flag to check if LIO-SAM odometry is received
    bool gazebo_odom_received = false;                 // Flag to check if Gazebo Ground Truth odometry is received
    bool initial_pose_saved = false;                   // Flag to check if the initial pose is saved
    bool initial_lio_pose_saved = false;               // Flag to check if the initial pose from LIO-SAM is saved
    bool initial_msg_received = false;                 // Flag to check if the initial message is received
    std::chrono::steady_clock::time_point start_time;  // Start time of the program
    nav_msgs::Odometry initial_pose;                   // Initial pose of the robot
    nav_msgs::Odometry initial_lio_pose;               // Initial pose of the robot from LIO-SAM

    /**
     * @brief Initialize Kalman filter
     *
     * Initialize the state vector, covariance matrix, state transition model, measurement model, measurement noise covariance, and process noise
     * covariance.
     */
    void initializeKalmanFilter() {
        state = Eigen::VectorXd(6);
        state = Eigen::VectorXd::Zero(6);
        covariance = Eigen::MatrixXd::Identity(6, 6);
        F = Eigen::MatrixXd::Identity(6, 6);
        // H = Eigen::MatrixXd::Zero(3, 6);
        // H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
        H = Eigen::MatrixXd::Identity(6, 6);
        // R = Eigen::MatrixXd::Identity(3, 3);
        R = Eigen::MatrixXd::Identity(6, 6);
        R = R * 0.01;
        // set first element of R to 0.2 to account for longitudinal tunnel effect
        R(0, 0) = 0.2;
        Q = Eigen::MatrixXd::Identity(6, 6);
        Q = Q * 0.01;
        // set first element of Q to 0.1 to account for longitudinal tunnel effect
        Q(0, 0) = 0.1;
    }

    /**
     * @brief Convert quaternion to Euler angles
     * @param q Quaternion
     * @return Euler angles
     *
     * Convert quaternion to Euler angles.
     */
    Eigen::Vector3d quaternionToEuler(const geometry_msgs::Quaternion& q) {
        tf::Quaternion quat(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    /**
     * @brief Callback function for LIO-SAM odometry
     * @param msg LIO-SAM odometry message
     *
     * Callback function for LIO-SAM odometry. Save the initial pose of the robot from LIO-SAM. Correct the odometry message with the initial pose.
     */
    void lioOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (initial_msg_received && std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            if (!initial_lio_pose_saved) {
                initial_lio_pose = *msg;
                initial_lio_pose_saved = true;
            }
            lio_odom = *msg;
            // correct liosam_odom with initial pose
            lio_odom.pose.pose.position.x -= initial_lio_pose.pose.pose.position.x;
            lio_odom.pose.pose.position.y -= initial_lio_pose.pose.pose.position.y;
            lio_odom.pose.pose.position.z -= initial_lio_pose.pose.pose.position.z;
            lio_odom.pose.pose.orientation.x -= initial_lio_pose.pose.pose.orientation.x;
            lio_odom.pose.pose.orientation.y -= initial_lio_pose.pose.pose.orientation.y;
            lio_odom.pose.pose.orientation.z -= initial_lio_pose.pose.pose.orientation.z;
            lio_odom.pose.pose.orientation.w -= initial_lio_pose.pose.pose.orientation.w;

            lio_odom_received = true;
            processOdometry();
        }
    }

    /**
     * @brief Callback function for Gazebo Ground Truth odometry
     * @param msg Gazebo Ground Truth odometry message
     *
     * Callback function for Gazebo Ground Truth odometry. Save the initial pose of the robot. Correct the odometry message with the initial pose.
     */
    void gazeboOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!initial_msg_received) {
            start_time = std::chrono::steady_clock::now();
            initial_msg_received = true;
        } else if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            if (!initial_pose_saved) {
                initial_pose = *msg;
                initial_pose_saved = true;
            }
            gazebo_odom = *msg;
            // correct gazebo_odom with initial pose
            gazebo_odom.pose.pose.position.x -= initial_pose.pose.pose.position.x;
            gazebo_odom.pose.pose.position.y -= initial_pose.pose.pose.position.y;
            gazebo_odom.pose.pose.position.z -= initial_pose.pose.pose.position.z;
            gazebo_odom.pose.pose.orientation.x -= initial_pose.pose.pose.orientation.x;
            gazebo_odom.pose.pose.orientation.y -= initial_pose.pose.pose.orientation.y;
            gazebo_odom.pose.pose.orientation.z -= initial_pose.pose.pose.orientation.z;
            gazebo_odom.pose.pose.orientation.w -= initial_pose.pose.pose.orientation.w;

            gazebo_odom_received = true;
            processOdometry();
        }
    }

    /**
     * @brief Process odometry messages
     *
     * Process odometry messages from LIO-SAM and Gazebo Ground Truth. Predict the state and covariance using the state transition model and process
     * noise covariance. Update the state and covariance using the measurement model and measurement noise covariance. Compute the pose error between
     * the two odometry messages. Print the pose error, covariance, measurement, ground truth, and state. Reset the flags for odometry messages.
     * If any of the measurements are NaN, skip this whole iteration.
     */
    void processOdometry() {
        if (lio_odom_received && gazebo_odom_received) {
            Eigen::VectorXd measurement(6);
            measurement << lio_odom.pose.pose.position.x, lio_odom.pose.pose.position.y, lio_odom.pose.pose.position.z,
                quaternionToEuler(lio_odom.pose.pose.orientation);

            // Print debug information
            // std::cout << "Measurement: " << measurement.transpose() << std::endl;
            // std::cout << "State before prediction: " << state.transpose() << std::endl;

            // If any of the measurements are NaN, skip this whole iteration
            if (measurement.hasNaN()) {
                ROS_WARN("Measurement has NaN values. Skipping this iteration.");
                lio_odom_received = false;
                gazebo_odom_received = false;
                return;
            }

            // Predict
            state = F * state;
            covariance = F * covariance * F.transpose() + Q;

            // Print debug information
            // std::cout << "State after prediction: " << state.transpose() << std::endl;
            // std::cout << "Covariance after prediction: " << std::endl << covariance << std::endl;

            // Update
            Eigen::VectorXd y = measurement - H * state;  // Ensure y is the same size as H * state
            // std::cout << "Measurement residual (y): " << y.transpose() << std::endl;

            Eigen::MatrixXd S = H * covariance * H.transpose() + R;
            // Eigen::MatrixXd S = H * covariance * H.transpose();

            // std::cout << "Residual covariance (S): " << std::endl << S << std::endl;

            Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();
            // std::cout << "Kalman gain (K): " << std::endl << K << std::endl;

            state = state + K * y;                                                // Update state (3x1)
            covariance = (Eigen::MatrixXd::Identity(6, 6) - K * H) * covariance;  // Update covariance

            // Print debug information
            // std::cout << "State after update: " << state.transpose() << std::endl;
            // std::cout << "Covariance after update: " << std::endl << covariance << std::endl;

            // Compute pose error
            Eigen::VectorXd ground_truth(6);
            ground_truth << gazebo_odom.pose.pose.position.x, gazebo_odom.pose.pose.position.y, gazebo_odom.pose.pose.position.z,
                quaternionToEuler(gazebo_odom.pose.pose.orientation);
            Eigen::VectorXd position_error = measurement.head<3>() - ground_truth.head<3>();
            Eigen::VectorXd orientation_error = measurement.tail<3>() - ground_truth.tail<3>();

            ROS_INFO_STREAM("Position Error: " << position_error.transpose() << "\nOrientation Error: " << orientation_error.transpose()
                                               << "\nCovariance: \n"
                                               << covariance << "\nMeasurement: " << measurement.transpose()
                                               << "\nGround Truth: " << ground_truth.transpose() << "\nState: " << state.transpose() << "\n");
            lio_odom_received = false;
            gazebo_odom_received = false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "covariance_estimator");
    CovarianceEstimator estimator;
    estimator.run();
    return 0;
}