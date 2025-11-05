#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "unomas/srv/query_soil.hpp"
#include "unomas/msg/soil_info.hpp"


#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <thread>
#include <cmath>


namespace Robot {
    class RobotController
    {
        public:

        //-------------------  CONSTRUCTOR AND DESTRUCTOR  -------------------//

            /*! @brief Constructor that allocates internals
             *
             *  @param[in] serial_id of robot
             *  @param[in] shared_pointer to a node
             */
            RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node);
            

            /*! @brief Destructor
             *
             */
            ~RobotController();

        //-------------------  FEEDBACK PUBLISHERS  -------------------//

            /*! @brief publisher callback that publishes the emergency state of the robot to the base station
             * 
             *  @return void
             */
            void publishEmergency();


            /*! @brief publisher callback that publishes battery level 
             * 
             *  @return void
             */
            void publishBattery();


            /*! @brief publisher callback that publishes the current position of the robot to the base station as a point
             * 
             *  @return void
             */
            void publishOdometry();


            /*! @brief publisher callback that publishes the current state of the robot to the base station
             *
             *  @param[in]    state - current state of the robot
             * 
             *  @return void
             */
            void publishState();


            /*! @brief publisher callback that publishes the connection status of the robot to the base station
             *
             *  @return void
             */
            void publishConnection();


            /*! @brief publisher callback that publishes the current goal of the robot to the base station
             *
             *  @param[in]    goal - current goal of the robot
             * 
             *  @return void
             */
            void publishGoal();      


            /*! @brief timer callback that calls feedback publishers
             *
             *  @return void
             */
            void timer_callback();

        //-------------------  FUNCTIONAL PUBLISHERS  -------------------//

            /*! @brief callback function to handle soildata service response and publishes soil data to basestation
             *
             *  @param[in] soilData - soil data response from query service
             * 
             *  @return void
             */
            void soilRequestCallback(rclcpp::Client<unomas::srv::QuerySoil>::SharedFuture future);


            /*! @brief publisher callback that publishes the goal_poses to the robot for Nav2
             *
             *  @param[in]    goal - goal to be reached
             * 
             *  @return void
             */
            void publishNavGoals(geometry_msgs::msg::Pose goal);


            /*! @brief publisher callback that publishes calculated velocities needed for the robot to move to cmd_vel
             *
             *  @param[in]    velocities - desired velocity of the robot
             * 
             *  @return void
             */
            void publishCmdVel(geometry_msgs::msg::Twist vel);

        //-------------------  FUNCTIONAL SUBSCRIBERS  -------------------//

            /*! @brief subscriber callback that listens for obstacles from detection node
             *
             *  @param[in]    obstacles - obstacles detected
             * 
             *  @return void
             */
            void subscribeObstacles(sensor_msgs::msg::LaserScan data);


            /*! @brief subscriber callback for ground facing lidar
             *
             *  @param[in]    laser - ground facing lidar data
             * 
             *  @return void
             */
            void subscribeGroundLiDAR(sensor_msgs::msg::LaserScan laser);


            /*! @brief sibscriber callback for robot odometry 
             *
             *  @param[in] odometry - updated and filtered odometry of robot
             * 
             *  @return void
             */
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


            /*! @brief subscriber callback for emergency return call
             *
             *  @param[in] point - point of emergency return
             * 
             *  @return void
             */
            void emergencyCallback(const geometry_msgs::msg::Point::SharedPtr msg);

        //------------------- MAIN THREAD -----------------//

            /*! @brief main threaded loop that runs the planner (5 states: travelling, surveying, aligning, sampling, emergency) (plus an "idle state")
             *
             *  @return void
             */
            void navThread();

        //------------------- OTHER FUNCTIONS -----------------//

            /*! @brief correct angle
             *
             *  @param[in] angle - angle to be corrected
             * 
             *  @return double - corrected angle
             */
            double correctAngle(double angle);


            /*! @brief find 2 closest points on each object
             *
             *  @param[in] closestPoints - vector of closest points
             * 
             *  @return vector of points
             */
            std::vector<geometry_msgs::msg::Point> twoClosest(std::vector<geometry_msgs::msg::Point> closestPoints);


            /*! @brief calculate allignment pose
             *
             *  @param[in] crops - vector of crops
             * 
             *  @return geometry_msgs::msg::Pose - allignment pose
             */
            geometry_msgs::msg::Point calculateAllignmentPoint(std::vector<geometry_msgs::msg::Point> crops);


            /*! @brief make local copy of lidar data
             *
             *  @return vector of points
             */
            sensor_msgs::msg::LaserScan copyLiDAR();


            /*! @brief convert raw lidar data into global vector of points
             *
             *  @param[in] lidar - vector of points
             * 
             *  @return vector of points
             */
            std::vector<geometry_msgs::msg::Point> processLiDAR(sensor_msgs::msg::LaserScan lidar);


            /*! @brief determine number of crop rows adjacent to the drone
             *
             *  @param[in] lidar - vector of points
             * 
             *  @return std::vector<geometry_msgs::msg::Point> - vector of points
             */
            std::vector<geometry_msgs::msg::Point> determineRows(std::vector<geometry_msgs::msg::Point> lidar);


            /*! @brief determine number of crop rows adjacent to the drone (only for use in surveying)
             *
             *  @param[in] lidar - vector of points
             * 
             *  @return std::vector<geometry_msgs::msg::Point> - vector of points
             */
            std::vector<geometry_msgs::msg::Point> locateInitialRows(std::vector<geometry_msgs::msg::Point> lidar);

            
            /*! @brief check if any known obstacles are within range of the drone
             *
             *  @param[in] point - point of obstacle
             * 
             *  @return bool - true if obstacle is in range
             */
            bool isObstacleInRange(geometry_msgs::msg::Point point);


            /*! @brief calls sample service, publishes soil data and updates last soil point
             *
             *  @return void
             */
            void querySoil();


            /*! @brief starts existing navThread if one exists and there are goals to complete 
             *
             *  @return bool - Returns true if vector of goals exists
             */
            bool autoNavigate();

        // ------------- GETTERS AND SETTERS -------------- //

            /*! @brief getter for Odometry
             * 
             *  @return odometry
             */
            nav_msgs::msg::Odometry getOdometry();


            /*! @brief getter for current goal
             *
             *  @return point of current goal 
             */
            geometry_msgs::msg::Pose getGoal();


            /*! @brief getter for current robot state
             *
             *  @return string of state
             */
            std::string getStatus();


            /*! @brief getter for emergency status of robot
             *
             *  @return bool - emegency status of robot
             */
            bool getEmergency();


            /*! @brief getter for battery level of robot
             *
             *  @return float - battery level
             */
            int getBattery();


            /*! @brief setter for goals from base station, will also create navThread 
             *
             *  @param[in] goals - vector of points
             * 
             *  @return void
             */
            void setGoals(geometry_msgs::msg::PoseArray goals);

        // ------------- VARIABLES ---------------- //

            std::string serial_id_;
            const std::shared_ptr<rclcpp::Node>& node_;
            // std::vector<geometry_msgs::msg::Point> goals_;
            // nav_msgs::msg::Odometry current_odometry_;
            nav_msgs::msg::Odometry last_query_odom_;
            // bool emergency_;
            // int battery_;
            // std::string current_status_;
            std::string registered_station_;

            // geometry_msgs::msg::Point goal_position_;

            const float minSampleDistance_ = 0.7;

            const float loop_rate_hz = 10.0;   // run loop 10 times per second

            std::chrono::duration<double> loop_period_;

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


            // feedback publishers
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_; //!< publisher for state
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyPub_; //!< publisher for emergency
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goalPub_; //!< publisher for current goal poses
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPub_; //!< publisher for battery level
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr odomPub_; //!< publisher for odometry
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connectionPub_; //!< publisher for connection status

            //rclcpp::WallTimer<CallbackT>::SharedPtr timer_; //!< timer for feedback publishers
            rclcpp::TimerBase::SharedPtr timer_; //!< timer for feedback publishers

            // functional publishers
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navPub_; //!< publisher for goal poses
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_; //!< publisher for cmd_vel
            rclcpp::Publisher<unomas::msg::SoilInfo>::SharedPtr soil_info_publisher_; //!< publisher for soil data

            // functional subscribers
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< subscriber for goal poses
            //rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstaclesSub_; //!< subscriber for obstacles
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr groundLiDARSub_; //!< subscriber for ground LiDAR
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; //!< subscriber for odometry
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr emergencySub_; //!< subscriber for emergency return call

            // service clients
            rclcpp::Client<unomas::srv::QuerySoil>::SharedPtr soil_query_client_; //!< client for soil query service


            struct positionData
            {
                std::mutex positionMutex;

                nav_msgs::msg::Odometry dronePose; //!< current position of the drone
            
            } positionData_; // position data


            struct feedbackData
            {
                std::mutex feedbackMutex; // mutex for feedback data

                bool emergency; // true if robot is in emergency
                float battery;   // battery level
                geometry_msgs::msg::Point currentPosition;   // current position of the robot
                std::string state; // current state of the robot 
                bool connection;   // true if robot is connected to base station
                geometry_msgs::msg::Point currentGoal; // current goal of the robot

                geometry_msgs::msg::Point noGoal; // Default pose if no goal is provided

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

                nav_msgs::msg::Odometry soilPose; // position of the last soil sample
                unomas::msg::SoilInfo soilData; // soil data message

            } soil_; // soil data


            struct goalsData
            {
                std::mutex goalsMutex;

                geometry_msgs::msg::PoseArray rawGoals; //!< vector of goal poses raw from base station
                geometry_msgs::msg::PoseArray droneGoals; //!< vector of goal poses that can be modified by the drone

                geometry_msgs::msg::Pose currentGoal; //!< current goal of the drone
                bool commuting = false;
                std::vector<double> pastDistances;
                int emergencyCounter;
                
            } goals_; // goals data

            
            struct threadData
            {
                std::mutex threadMutex; 

                std::thread* navThread; //!< thread for the navigation
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
                

                // aligning
                State aligningState;
                geometry_msgs::msg::Point checkpoint;

                // waiting

                // emergency
                bool emergencyReturn; // true if robot is in emergency return
                geometry_msgs::msg::Point emergencyPosition; // position of emergency return

                // universal
                std::mutex stateMutex;

                State superState;
                bool changedState;
                std::atomic_bool navDone; //!< atomic boolean for the navigation thread
                
            } stateData_;


            struct cropData
            {
                std::mutex cropMutex;

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

    };
}

#endif // ROBOT_INTERFACE_H