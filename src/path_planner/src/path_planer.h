#ifndef PATH_PLANER_H
#define PATH_PLANER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

//#include <custom_msgs/SoilData.h>
//#include <custom_msgs/Obstacles.h>

#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <thread>
#include <cmath>


/*!
* @brief PathPlanner class
* @author Connor
*
* This class controls the path planning and control logic of the robots movement
*
*/
class PathPlanner : public rclcpp::Node
{
public:

//-------------------  CONSTRUCTOR AND DESTRUCTOR  -------------------//

    /*! @brief Constructor that allocates internals
     *
     *  @param[in] array of goal poses
     */
    PathPlanner();


    /*! @brief Destructor
     *
     */
    ~PathPlanner();


private:

//-------------------  FEEDBACK PUBLISHERS  -------------------//

    /*! @brief publisher callback that publishes the current goal of the robot to the base station
     *
     *  @param[in]    goal - current goal of the robot
     * 
     *  @return void
     */
    void publishGoal();


    /*! @brief publisher callback that publishes the current state of the robot to the base station
     *
     *  @param[in]    state - current state of the robot
     * 
     *  @return void
     */
    void publishState();


    /*! @brief publisher callback that publishes the emergency state of the robot to the base station
     *
     *  @param[in]    emergency - current emergency state of the robot
     * 
     *  @return void
     */
    void publishEmergency();

//-------------------  FUNCTIONAL PUBLISHERS  -------------------//

    /*! @brief publisher callback that publishes the goal_poses to the robot for Nav2
     *
     *  @param[in]    goal - goal to be reached
     * 
     *  @return void
     */
    void publishNavGoals();


    /*! @brief publisher callback that publishes calculated velocities needed for the robot to move to cmd_vel
     *
     *  @param[in]    velocities - desired velocity of the robot
     * 
     *  @return void
     */
    void publishCmdVel(geometry_msgs::msg::Twist vel);


    /*! @brief publisher callback that publishes the collected soil data to the base station
     *
     *  @param[in]    soil - soil data
     * 
     *  @return void
     */
    void publishPose();


    /*! @brief timer callback that calls feedback publishers
     *
     *  @return void
     */
    void timer_callback();

//-------------------  FUNCTIONAL SUBSCRIBERS  -------------------//

    /*! @brief subscriber callback that listens for goal pose instruction from base station
     *
     *  @param[in]    goals - goals to be reached
     * 
     *  @return void
     */
    void subscribeGoals(geometry_msgs::msg::PoseArray goals);


    /*! @brief subscriber callback that listens for obstacles from detection node
     *
     *  @param[in]    obstacles - obstacles detected
     * 
     *  @return void
     */
    void subscribeObstacles(sensor_msgs::msg::LaserScan data);

//-------------------  OTHER FUNCTIONS AND THREADS -----------------//

    /*! @brief main threaded loop that runs the planner (4 states: travelling, aligning, sampling, waiting, emergency)
     *
     *  @return void
     */
    void navThread();


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


    /*! @brief calls sample service, publishes soil data and updates last soil point
     *
     *  @return bool - true if successful
     */
    bool sample();


private:

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

    geometry_msgs::msg::Pose dronePose_; //!< next goal of the drone  //!(could be replaced with odo) !//

    // feedback publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_; //!< publisher for state
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyPub_; //!< publisher for emergency
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub_; //!< publisher for current goal poses

    //rclcpp::WallTimer<CallbackT>::SharedPtr timer_; //!< timer for feedback publishers
    rclcpp::TimerBase::SharedPtr timer_; //!< timer for feedback publishers

    // functional publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navPub_; //!< publisher for goal poses
    rclcpp::Publisher<custom_msgs::SoilData>::SharedPtr soilPub_; //!< publisher for soil data
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_; //!< publisher for cmd_vel

    // functional subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< subscriber for goal poses
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstaclesSub_; //!< subscriber for obstacles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr groundLiDARSub_; //!< subscriber for ground LiDAR
    

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

        geometry_msgs::msg::Pose soilPose; // position of the last soil sample
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
        double minSampleDistance = 1.0;

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

    const float minDistObjects = 0.4; //!< minimum distance to distinguish consecutive points between objects
    const float minSampleDistance = 0.7;  //!< minimum distance between samples (also used to determine if allignment point is in already sampled row)



};

#endif