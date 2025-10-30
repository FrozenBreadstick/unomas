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


#endif

/*!
* @brief PathPlanner class
* @author Connor
*
* This class controls the path planning and control logic of the robots movement
*
*/
class PathPlanner
{
public:

//-------------------  CONSTRUCTOR AND DESTRUCTOR  -------------------//

    /*! @brief Constructor that allocates internals
     *
     *  @param[in] array of goal poses
     */
    PathPlanner(geometry_msgs::msg::PoseArray goals);


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
    void publishGoals();


    /*! @brief publisher callback that publishes calculated velocities needed for the robot to move to cmd_vel
     *
     *  @param[in]    velocities - desired velocity of the robot
     * 
     *  @return void
     */
    void publishVelocities();


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
    void subscribeObstacles(custom_msgs::Obstacles obstacles);

//-------------------  OTHER FUNCTIONS AND THREADS -----------------//

    /*! @brief main threaded loop that runs the planner (4 states: travelling, aligning, sampling, waiting, emergency)
     *
     *  @return void
     */
    void navThread();


    /*! @brief detect crops
     *
     *  @return vector of points of crops
     */
    std::vector<geometry_msgs::msg::Point> detectCrops();

    /*! @brief determine distinct objects from an array of points
     *
     *  @param[in] crops - vector of crops
     * 
     *  @return vector of vectors of points
     */
    std::vector<std::vector<geometry_msgs::msg::Point>> determineObjects(std::vector<geometry_msgs::msg::Point> crops);

    /*! @brief find the closest point on each object
     *
     *  @param[in] objects - vector of vectors of points
     * 
     *  @return vector of points
     */
    std::vector<geometry_msgs::msg::Point> findClosestPoints(std::vector<std::vector<geometry_msgs::msg::Point>> objects);

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

    /*! @brief correct angle
     *
     *  @param[in] angle - angle to be corrected
     * 
     *  @return double - corrected angle
     */
    double correctAngle(double angle);


    /*! @brief get raw object data
     *
     *  @return vector of points
     */
    std::vector<geometry_msgs::msg::Point> getRawObjectData();

    /*! @brief get goal data
     *
     *  @return vector of poses
     */
    std::vector<geometry_msgs::msg::Pose> getGoalData();

private:

    enum State
    {
        TRAVELLING,
        SURVEYING,
        ALIGNING,
        SAMPLING,
        IDLE,
        EMERGENCY

    } state_; //!< current state of the robot
    

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

        std::vector<geometry_msgs::msg::Point> objectPoints;   // laser scanned points, and the object they are part of (i.e. crop1, crop2) (could probably make an enum for this) (might need to be a custom message)
   
    } objects_; // vector of object points


    struct groundLiDARData
    {
        std::mutex groundLiDARMutex;

        sensor_msgs::msg::LaserScan data;

    } groundLiDAR_; // vector of object points
 

    struct soilData
    {
        std::mutex soilMutex;

        geometry_msgs::msg::Pose soilPose; // position of the soil
        custom_msgs::SoilData soilData; // soil data

        std::vector<geometry_msgs::msg::Point> sampledPoints; // vector of points that have been sampled

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

    geometry_msgs::msg::Pose robotPose_; //!< next goal of the drone  //!(could be replaced with odo) !//

    // feedback publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub_; //!< publisher for state
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyPub_; //!< publisher for emergency
    rclcpp::Publisher<geometry_msgs/msg::PoseStamped>::SharedPtr goalPub_; //!< publisher for current goal poses

    rclcpp::WallTimer<CallbackT>::SharedPtr timer_; //!< timer for feedback publishers


    // functional publishers
    rclcpp::Publisher<geometry_msgs/msg/PoseStamped>::SharedPtr navPub_; //!< publisher for goal poses
    rclcpp::Publisher<custom_msgs::SoilData>::SharedPtr soilPub_; //!< publisher for soil data
    rclcpp::Publisher<geometry_msgs/msg/Twist>::SharedPtr cmdVelPub_; //!< publisher for cmd_vel

    // functional subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< subscriber for goal poses
    rclcpp::Subscription<custom_msgs::Obstacles>::SharedPtr obstaclesSub_; //!< subscriber for obstacles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr groundLiDARSub_; //!< subscriber for ground LiDAR

    struct threadData
    {
        std::thread navThread; //!< thread for the navigation
        atomic_bool navDone; //!< atomic boolean for the navigation thread
        atomic_bool threadExists; //!< atomic boolean for the navigation thread

    } threadData_; //!< thread data


    struct manuelNavData
    {
        const float rotate = 0.1;
        const float linear = 0.1;

    } vel_; //!< manuel nav data


    struct stateData
    {
        // surveying
        SurveyingState surveyingState;
        double initSurveyingAngle;
        bool startedRotating = false;

        // sampling
        bool endSampling;
        int noCropsCounter;
        bool exitMove;
        unsigned double moveTimer;

        // aligning

        // waiting

        // emergency

        // universal
        bool changedState;
        
    } stateData_;

    
    enum SurveyingState
    {
        ROTATING,
        CALCULATING,
        MOVING,
        EXITING
    }


    struct cropData
    {
        std::vector<geometry_msgs::msg::Point> cropCentres;

        geometry_msgs::msg::Point currentCrop;
        geometry_msgs::msg::Point nextCrop;

        geometry_msgs::msg::Point rowCentre;

    } cropData_;

    const float minDistObjects = 0.4; //!< minimum distance to distinguish consecutive points between objects
    const float minSampleDistance = 0.7;  //!< minimum distance between samples (also used to determine if allignment point is in already sampled row)



}