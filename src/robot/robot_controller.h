#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "unomas/srv/query_soil.hpp"
#include "unomas/msg/soil_info.hpp"

namespace Robot {
    class RobotController
    {
        public:
            RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node);
            ~RobotController();

            nav_msgs::msg::Odometry getOdometry();

            geometry_msgs::msg::Point getGoal();

            std::string getStatus();

            bool isEmergency();

            int getBattery();

            void sendCmd(geometry_msgs::msg::Twist cmd);

            void setGoals(std::vector<geometry_msgs::msg::Point> goals);

            void querySoil();

            bool autoNavigate(); // Returns true if vector of goals exists, function to use NAV2 to move between goals in goals_ vector
            // Use threads so that it doesnt block the main uplink nodes subscribers and publishers

        private:
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void soilRequestCallback(rclcpp::Client<unomas::srv::QuerySoil>::SharedFuture future);

            std::string serial_id_;
            const std::shared_ptr<rclcpp::Node>& node_;
            std::vector<geometry_msgs::msg::Point> goals_;
            nav_msgs::msg::Odometry current_odometry_;
            nav_msgs::msg::Odometry last_query_odom_;
            bool emergency_;
            int battery_;
            std::string current_status_;
            std::string registered_station_;

            geometry_msgs::msg::Point goal_position_;

            /* Variables for auto navigation and path planning */
            enum class State
            {
                // SUPER STATES
                TRAVELLING,
                SURVEYING,
                ALIGNING,
                SAMPLING,
                IDLE,
                EMERGENCY,

                // SURVEYING SUB STATES
                ROTATING,
                CALCULATING,
                S_MOVING,
                EXITING,

                // ALLIGNING SUB STATES
                LEAVING,
                TURNING_PERP,
                TURNING_PARA,
                A_MOVING,
                ENTERING,
                CHECKING
            }; 

            nav_msgs::msg::Odometry dronePose_; //!< next goal of the drone  //!(could be replaced with odo) !//

            // feedback publishers
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_; //!< publisher for state
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyPub_; //!< publisher for emergency
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_; //!< publisher for current goal poses

            //rclcpp::WallTimer<CallbackT>::SharedPtr timer_; //!< timer for feedback publishers
            rclcpp::TimerBase::SharedPtr timer_; //!< timer for feedback publishers

            // functional publishers
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navPub_; //!< publisher for goal poses
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; //!< publisher for cmd_vel
            rclcpp::Publisher<unomas::msg::SoilInfo>::SharedPtr soil_info_publisher_; //!< publisher for soil data

            // functional subscribers
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< subscriber for goal poses
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstaclesSub_; //!< subscriber for obstacles
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr groundLiDARSub_; //!< subscriber for ground LiDAR
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; //!< subscriber for odometry

            // service clients
            rclcpp::Client<unomas::srv::QuerySoil>::SharedPtr soil_query_client_; //!< client for soil query service

            struct feedbackData
            {
                std::mutex feedbackMutex; // mutex for feedback data

                bool emergency; // true if robot is in emergency
                std::string state; // current state of the robot (might need to be a string)
                geometry_msgs::msg::Pose goal; // current goal of the robot

            } feedbackData_; //!< feedback data structure containing the progress of the motion, the status of the motion and the current pose


            struct objectData
            {
                std::mutex objectMutex;

                geometry_msgs::msg::PoseArray objectPoints;   // obstacle points from ayberk (might need to be a custom message)

                double minRange = 0.4;
        
            } objects_; // vector of object points


            struct groundLiDARData
            {
                std::mutex groundLiDARMutex;

                sensor_msgs::msg::LaserScan data;   // data from the ground facing LiDAR
                geometry_msgs::msg::Point offset;   // offset of the ground facing LiDAR from the drone

            } groundLiDAR_; // vector of object points
        

            struct soilData
            {
                std::mutex soilMutex;

                geometry_msgs::msg::Point soilPose; // position of the last soil sample
                custom_msgs::SoilData soilData; // soil data message

                

            } soil_; // soil data


            struct goalsData
            {
                std::mutex goalsMutex;

                geometry_msgs::msg::PoseArray rawGoals; //!< vector of goal poses raw from base station
                std::vector<geometry_msgs::msg::Pose> droneGoals; //!< vector of goal poses that can be modified by the drone

                geometry_msgs::msg::Pose currentGoal; //!< current goal of the drone
                bool commuting = false;
                std::vector<double> pastDistances;
                int emergencyCounter;
                
            } goals_; // goals data

            
            struct threadData
            {
                std::thread* navThread; //!< thread for the navigation
                std::atomic_bool navDone; //!< atomic boolean for the navigation thread
                std::atomic_bool threadExists; //!< atomic boolean for the navigation thread

            } threadData_; //!< thread data


            struct manualNavData
            {
                const float rotate = 0.1;
                const float linear = 0.1;

            } manualNavData_; //!< manual nav data


            struct stateData
            {
                // surveying
                State surveyingState;
                double initSurveyingAngle;
                bool startedRotating = false;
                bool rowAlligned = false;

                // sampling
                const float minSampleDistance = 0.7;  //!< minimum distance between samples (also used to determine if allignment point is in already sampled row)

                // aligning
                State aligningState;
                geometry_msgs::msg::Point checkpoint;

                // waiting

                // emergency

                // universal
                State superState;
                bool changedState;
                
            } stateData_;


            struct cropData
            {
                std::vector<geometry_msgs::msg::Point> cropCentres;

                geometry_msgs::msg::Point currentCrop;
                geometry_msgs::msg::Point nextCrop;

                geometry_msgs::msg::Point rowCentre;

                geometry_msgs::msg::Point rowCorner;

                geometry_msgs::msg::Point rowParallel;
                geometry_msgs::msg::Point rowPerpendicular;

                double midRowScaler;
                geometry_msgs::msg::Point midRowVector;

                bool leftTurnRow = true;

                double bumpThreshold = 0.1;

            } cropData_;

}

#endif // ROBOT_INTERFACE_H