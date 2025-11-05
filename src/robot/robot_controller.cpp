#include "robot_controller.h"

/*

    Things that need doing

    - General error checking and building and testing

*/

// clock for loop checking
using robotClock = std::chrono::steady_clock;
using namespace std::chrono_literals;     

//-------------------  CONSTRUCTOR AND DESTRUCTOR  -------------------//

Robot::RobotController::RobotController(std::string serial_id, const std::shared_ptr<rclcpp::Node>& node) : 
    serial_id_(serial_id), 
    node_(node),
    registered_station_("")
{

    // std::string cmd_topic = "/cmd_vel"; //In a multi robot setup this would be combined with serial_id as a prefix
    // cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    // std::string odom_topic = "/odometry/filtered"; //Filtered uses the IMU to correct drift (better for NAV2)
    // odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    //     odom_topic, 10, std::bind(&Robot::RobotController::odomCallback, this, std::placeholders::_1)
    // );

    // set rogue variables   
    groundLiDAR_.offset.x = 0.12374;
    groundLiDAR_.offset.y = 0.0;
    groundLiDAR_.offset.z = 0.12374;
    feedbackData_.noGoal.x = 0;
    feedbackData_.noGoal.y = 0;
    feedbackData_.noGoal.z = -1;

    feedbackData_.emergency = false;
    feedbackData_.battery = 100;
    feedbackData_.state = "IDLE";
    feedbackData_.currentGoal = feedbackData_.noGoal;
    feedbackData_.currentPosition.x = 0;
    feedbackData_.currentPosition.y = 0;
    feedbackData_.currentPosition.z = 0;

    stateData_.superState = State::IDLE;

    // timer
    loop_period_ = std::chrono::duration<double>(1.0 / loop_rate_hz);

    // initialise feedback publishers   
    std::string state_topic = "/state";
    statePub_ = node_->create_publisher<std_msgs::msg::String>(state_topic, 10);
    std::string emergency_topic = "/emergency";
    emergencyPub_ = node_->create_publisher<std_msgs::msg::Bool>(emergency_topic, 10);
    std::string goal_topic = "/goal";
    goalPub_ = node_->create_publisher<geometry_msgs::msg::Point>(goal_topic, 10);
    std::string battery_topic = "/battery";
    batteryPub_ = node_->create_publisher<std_msgs::msg::Float32>(battery_topic, 10);
    std::string position_topic = "/odom";
    odomPub_ = node_->create_publisher<geometry_msgs::msg::Point>(position_topic, 10);
    std::string connection_topic = "/connection";
    connectionPub_ = node_->create_publisher<std_msgs::msg::Bool>(connection_topic, 10);

    // initialise publishers
    std::string nav_topic = "/goal_pose";
    navPub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(nav_topic, 10);
    std::string soil_topic = "/soil";
    soil_info_publisher_ = node_->create_publisher<unomas::msg::SoilInfo>(soil_topic, 10);
    std::string cmd_vel_topic = "/cmd_vel"; //In a multi robot setup this would be combined with serial_id as a prefix
    cmdVelPub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // initialise subscribers
    // std::string goals_topic = "/goals";
    // goalsSub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(goals_topic, 10, std::bind(&Robot::RobotController::subscribeGoals, this, std::placeholders::_1));
    // std::string obstacles_topic = "/obstacles";
    // obstaclesSub_ = node_->create_subscription<custom_msgs::Obstacles>(obstacles_topic, 10, std::bind(&Robot::RobotController::subscribeObstacles, this, std::placeholders::_1));
    std::string groundLiDAR_topic = "/laserscan2";
    groundLiDARSub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(groundLiDAR_topic, 10, std::bind(&Robot::RobotController::subscribeGroundLiDAR, this, std::placeholders::_1));
    std::string odom_topic = "/odometry/filtered"; //Filtered uses the IMU to correct drift (better for NAV2)
    odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&Robot::RobotController::odomCallback, this, std::placeholders::_1));
    std::string E_Return_topic = "/emergency_return";
    emergencySub_ = node_->create_subscription<geometry_msgs::msg::Point>(E_Return_topic, 10, std::bind(&Robot::RobotController::emergencyCallback, this, std::placeholders::_1));

    // initialise wall timer for feedback pubs
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Robot::RobotController::timer_callback, this));

    // initialise service clients
    soil_query_client_ = node_->create_client<unomas::srv::QuerySoil>("query_soil");

    soil_info_publisher_ = node_->create_publisher<unomas::msg::SoilInfo>("unomas/soil", 10);

}

Robot::RobotController::~RobotController()
{
    RCLCPP_INFO(node_->get_logger(), "RobotController for robot '%s' is shutting down.", serial_id_.c_str());

    // close nav thread
    std::lock_guard<std::mutex> lock(threadData_.threadMutex);

    if(threadData_.threadExists == true) {
        if(threadData_.navThread->joinable() == true) {
            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

            stateData_.navDone = true;
            threadData_.navThread->join();
            delete threadData_.navThread;
        }
    }
}

//-------------------  FEEDBACK PUBLISHERS  -------------------//


// publisher callback that publishes the current state of the robot to the base station
void Robot::RobotController::publishState()
{
    // make message
    std_msgs::msg::String msg;

    msg.data = feedbackData_.state;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing %s", msg.data.c_str());

    // publish message
    statePub_->publish(msg);


}


// publisher callback that publishes the emergency state of the robot to the base station
void Robot::RobotController::publishEmergency()
{
    // make message
    std_msgs::msg::Bool msg;

    msg.data = feedbackData_.emergency;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing %s", msg.data ? "true" : "false");

    // publish message
    emergencyPub_->publish(msg);
}


// publisher callback that publishes the current goal of the robot to the base station
void Robot::RobotController::publishGoal()
{
    // make message
    geometry_msgs::msg::Point msg;

    msg = feedbackData_.currentGoal;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing goal: (%.2f, %.2f)", msg.x, msg.y);

    // publish message
    goalPub_->publish(msg);
}


// publisher callback that publishes battery level
void Robot::RobotController::publishBattery()
{
    // make message
    std_msgs::msg::Float32 msg;

    msg.data = feedbackData_.battery;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing battery level: %.2f", msg.data);

    // publish message
    batteryPub_->publish(msg);
}


// publisher callback that publishes odometry
void Robot::RobotController::publishOdometry()
{
    // make message
    geometry_msgs::msg::Point msg;

    feedbackData_.currentPosition = getOdometry().pose.pose.position;

    msg = feedbackData_.currentPosition;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing odometry: (%.2f, %.2f, %.2f)", msg.x, msg.y, msg.z);

    // publish message
    odomPub_->publish(msg);
}


// publisher callback that publishes connection status
void Robot::RobotController::publishConnection()
{
    // make message
    std_msgs::msg::Bool msg;

    msg.data = feedbackData_.connection;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing connection status: %s", msg.data ? "true" : "false");

    // publish message
    connectionPub_->publish(msg);
}


// timer callback that calls feedback publishers
void Robot::RobotController::timer_callback()
{
    // lock feedback mutex
    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

    // publish state
    publishState();

    // publish emergency
    publishEmergency();

    // publish goal
    publishGoal();

    // publish battery
    publishBattery();

    // publish odometry
    publishOdometry();

    // publish connection
    publishConnection();
}


//-------------------  FUNCTIONAL PUBLISHERS  -------------------//


// service callback that publishes soil data
void Robot::RobotController::soilRequestCallback(rclcpp::Client<unomas::srv::QuerySoil>::SharedFuture future)
{
    auto response = future.get();

    unomas::msg::SoilInfo soil_info_msg;
    soil_info_msg.x = last_query_odom_.pose.pose.position.x;
    soil_info_msg.y = last_query_odom_.pose.pose.position.y;
    soil_info_msg.moisture = response->moisture;
    soil_info_msg.nutrients = response->nutrients;
    soil_info_msg.ph = response->ph;
    soil_info_publisher_->publish(soil_info_msg);
}


// publisher callback that publishes velocity commands
void Robot::RobotController::publishCmdVel(geometry_msgs::msg::Twist vel)
{
    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing Twist linear=(%.2f, %.2f, %.2f), angular=(%.2f, %.2f, %.2f)", vel.linear.x, vel.linear.y, vel.linear.z, vel.angular.x, vel.angular.y, vel.angular.z);

    // publish message
    cmdVelPub_->publish(vel);

}


// publisher callback that publishes goal poses to Nav2
void Robot::RobotController::publishNavGoals(geometry_msgs::msg::Pose goal)
{
    // make message
    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = node_->now();
    msg.header.frame_id = "/world";
    msg.pose = goal;

    // log message
    RCLCPP_INFO(node_->get_logger(), "Publishing Nav goal (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y);

    // publish message
    navPub_->publish(msg);
}


//-------------------  FUNCTIONAL SUBSCRIBERS  -------------------//

// This whole subscriber needs fixing
// subscriber callback that listens for obstacles from detection node
// void Robot::RobotController::subscribeObstacles(geometry_msgs::msg::PoseArray obstacles)
// {
//     // lock mutex and save data
//     {
//     std::lock_guard<std::mutex> lock(objects_.objectMutex);
    
//     objects_.objectPoints = obstacles.poses;
//     }
// }


// subscriber callback that listens for ground LiDAR
void Robot::RobotController::subscribeGroundLiDAR(sensor_msgs::msg::LaserScan laser)
{
    // lock mutex and save laser data
    {
    std::lock_guard<std::mutex> lock(groundLiDAR_.groundLiDARMutex);
    
    groundLiDAR_.data = laser;
    }
}


// callback for odometry
void Robot::RobotController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(positionData_.positionMutex);

    positionData_.dronePose = *msg;
}


// callback for emergency return
void Robot::RobotController::emergencyCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // lock feedback mutex
    {
        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

        stateData_.emergencyReturn = true;
        stateData_.emergencyPosition = *msg;
        stateData_.changedState = true;
    }

    RCLCPP_INFO(node_->get_logger(), "Emergency return to (%.2f, %.2f)", msg->x, msg->y);
}


//-------------------  MAIN THREADED LOOP  -----------------//


// main threaded loop that runs the planner (4 states: travelling, aligning, sampling, emergency)
void Robot::RobotController::navThread()
{

    bool threadExists;
    bool navDone;

    {
        std::lock_guard<std::mutex> lock(threadData_.threadMutex);

        threadExists = threadData_.threadExists;
    }

    while(threadExists == true) {

        {
        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

        navDone = stateData_.navDone;
        }

        while(navDone == false)
        {

            // clock loop start
            auto loop_start = robotClock::now();

            // local copy of odometry               
            nav_msgs::msg::Odometry dronePose_ = RobotController::getOdometry();

            //lock feedback mutex
            bool emergencyReturn;

            {
                std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                emergencyReturn = stateData_.emergencyReturn;
            }
            

            if(emergencyReturn == false) {

                State currentState;

                {
                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                    currentState = stateData_.superState;
                }

                // do state actions
                switch(currentState)
                {
                    // travelling to field guess
                    case State::TRAVELLING: {

                        // lock goals mtuex
                        {
                            std::lock_guard<std::mutex> lock(goals_.goalsMutex);

                            // publish to nav2 and moniter status
                            if(goals_.droneGoals.poses.size() > 0) {

                                // check if drone is already moving to a goal
                                if(goals_.commuting == false) {
                                    // set goal
                                    goals_.currentGoal = goals_.droneGoals.poses.at(0);
                                    goals_.commuting = true;

                                    /* ...Publish goal to nav2... */
                                    publishNavGoals(goals_.currentGoal);
                                }
                                else {

                                    double distance = sqrt(pow(goals_.currentGoal.position.x - dronePose_.pose.pose.position.x, 2) + pow(goals_.currentGoal.position.y - dronePose_.pose.pose.position.y, 2));

                                    // check goal has been reached
                                    if(distance < 0.05) {
                                        // set new goal
                                        goals_.droneGoals.poses.erase(goals_.droneGoals.poses.begin());
                                        goals_.commuting = false;
                                        goals_.emergencyCounter = 0;
                                        goals_.pastDistances.clear();
                                    }
                                    else {
                                        // ensure drone is still moving towards goal
                                        if(goals_.pastDistances.size() > 20) {
                                            for (auto pastDistance : goals_.pastDistances) {
                                                if (distance - pastDistance > 0.05) {
                                                    goals_.commuting = false;   // reset commuting
                                                    goals_.emergencyCounter++;
                                                }
                                                else {
                                                    goals_.emergencyCounter = 0;
                                                }
                                            }
                                        }
                                        // if counter is greater than 10 then move to emergency
                                        if(goals_.emergencyCounter > 10) {

                                            // lock feedback and state mutex
                                            {
                                                std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                                                stateData_.superState = State::EMERGENCY;
                                                stateData_.changedState = true;
                                            }
                                        
                                            {
                                                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);
                                                feedbackData_.state = "EMERGENCY";
                                            }
                                            break;
                                        }

                                        // add and remove past distances
                                        goals_.pastDistances.push_back(distance);
                                        if(goals_.pastDistances.size() > 20) {
                                            goals_.pastDistances.erase(goals_.pastDistances.begin());
                                        }
                                    }
                                }
                            }
                            else {
                                {
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                                    stateData_.superState = State::ALIGNING;
                                }

                                {
                                    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);
                                    stateData_.changedState = true;
                                    feedbackData_.state = "ALIGNING";
                                }
                            }

                        }

                        }

                        break;

                    // surveying
                    case State::SURVEYING: {

                        State surveyState;

                        {
                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                            surveyState = stateData_.surveyingState;
                        }   

                        switch(surveyState)
                        {
                            // do a rotation and identify closest (starting row/current row) and second closest (next row) row centre to beacon
                            case State::ROTATING: {

                                // if first loop grab initial bearing
                                bool changedState;

                                {
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    changedState = stateData_.changedState;
                                }

                                if(changedState == true) {

                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    stateData_.changedState = false;

                                    stateData_.initSurveyingAngle = dronePose_.pose.pose.orientation.z;
                                    stateData_.startedRotating = false;
                                }
                                else {
                                
                                    // identify bumps and save points in crop centres if there are at least 2 bumps
                                    std::vector<geometry_msgs::msg::Point> rows = locateInitialRows(processLiDAR(copyLiDAR()));

                                    if(rows.size() > 1) {
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        cropData_.cropCentres = rows;
                                    }

                                    // rotate robot
                                    geometry_msgs::msg::Twist vel;
                                    vel.angular.z = manualNavData_.rotate;
                                    cmdVelPub_->publish(vel);

                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    // check if robot has rotated enough
                                    if(dronePose_.pose.pose.orientation.z - stateData_.initSurveyingAngle < 0.1) {
                                        
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        // ensure it has identified at least 2 bumps
                                        if(cropData_.cropCentres.size() > 1) {

                                            // ensure it has started rotating
                                            if(stateData_.startedRotating == true) {

                                                stateData_.surveyingState = State::CALCULATING;
                                            }
                                        }
                                        else {
                                            stateData_.startedRotating = false;
                                        }
                                    }
                                    else {
                                        stateData_.startedRotating = true;
                                    }
                                }

                                }

                                break;

                        // calculate middle of row, calculate direction and distance to move to middle of row (i.e. direction of line, move half distance of line)
                            case State::CALCULATING: {

                                std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                // find closest 2 crop centres
                                std::vector<geometry_msgs::msg::Point> crops = twoClosest(cropData_.cropCentres); //alternatively use this to make a local copy of the var

                                cropData_.currentCrop = crops.at(0);
                                cropData_.nextCrop = crops.at(1);

                                // find row centre
                                cropData_.rowCentre = calculateAllignmentPoint(crops);

                                // calculate crop field side direction vectors
                                cropData_.rowParallel.x = cropData_.currentCrop.x - dronePose_.pose.pose.position.x;
                                cropData_.rowParallel.y = cropData_.currentCrop.y - dronePose_.pose.pose.position.y;
                                cropData_.rowParallel.z = 0;
                                
                                double magnitude = sqrt(pow(cropData_.rowParallel.x, 2) + pow(cropData_.rowParallel.y, 2));
                                cropData_.rowParallel.x = cropData_.rowParallel.x / magnitude;
                                cropData_.rowParallel.y = cropData_.rowParallel.y / magnitude;
                        
                                cropData_.rowPerpendicular.x = -1 * cropData_.rowParallel.y;
                                cropData_.rowPerpendicular.y = cropData_.rowParallel.x;

                                // calculate projection of crop row centre onto crop field perpendicular direction vector
                                geometry_msgs::msg::Point rowCentreProj;
                                rowCentreProj.x = cropData_.rowCentre.x - dronePose_.pose.pose.position.x;
                                rowCentreProj.y = cropData_.rowCentre.y - dronePose_.pose.pose.position.y;
                                rowCentreProj.z = 0;

                                cropData_.midRowScaler = rowCentreProj.x * cropData_.rowPerpendicular.x + rowCentreProj.y * cropData_.rowPerpendicular.y;
                                cropData_.midRowVector.x = cropData_.rowPerpendicular.x * cropData_.midRowScaler;
                                cropData_.midRowVector.y = cropData_.rowPerpendicular.y * cropData_.midRowScaler;

                                // change sub-state
                                {
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    stateData_.surveyingState = State::S_MOVING;
                                }

                                }

                                break;
                            
                            // move forward to row middle and turn to face down row (rotate 90 to the right)
                            case State::S_MOVING: {

                                bool rowAlligned;
                                {
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    rowAlligned = stateData_.rowAlligned;
                                }

                                geometry_msgs::msg::Twist vel;

                                if(rowAlligned == false) {
                                    std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                    // calculate angle between rowPerpendicular and drone
                                    double angle = atan2(cropData_.rowPerpendicular.y, cropData_.rowPerpendicular.x);
                                    angle = correctAngle(angle);
                                    int direction = angle / abs(angle);

                                    // calculate magnitude of distance from corner to drone
                                    double distance = sqrt(pow(dronePose_.pose.pose.position.x - cropData_.rowCorner.x, 2) + pow(dronePose_.pose.pose.position.y - cropData_.rowCorner.y, 2));

                                    // check if drone is facing perp to row and close to mid row
                                    if(dronePose_.pose.pose.orientation.z - angle > 0.1) {

                                        vel.angular.z = direction * manualNavData_.rotate;
                                    }
                                    else if(distance - cropData_.midRowScaler > 0.1) {
                                        vel.linear.x = manualNavData_.linear;
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.rowAlligned = true;
                                    }
                                }
                                else {
                                    std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                    // calculate angle between rowParallel and drone
                                    double angle = atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                                    angle = correctAngle(angle);
                                    int direction = angle / abs(angle);

                                    // check if drone is facing down row
                                    if(dronePose_.pose.pose.orientation.z - angle > 0.1) {
                                        vel.angular.z = direction * manualNavData_.rotate;
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                                        
                                        stateData_.surveyingState = State::EXITING;
                                    }
                                }

                                cmdVelPub_->publish(vel);

                                }

                                break;

                            // switch to sampling
                            case State::EXITING: {

                                {
                                    // reset state data
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                                    stateData_.superState = State::SAMPLING;
                                    stateData_.surveyingState = State::ROTATING;
                                    stateData_.changedState = true;
                                    stateData_.rowAlligned = false;
                                }

                                {
                                    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);
                                    feedbackData_.state = "SAMPLING";
                                }
                            
                                }

                                break;

                            // default
                            default: 

                                break;
                        }

                        }

                        break;

                    // aligning
                    case State::ALIGNING: {

                        bool changedState;
                        geometry_msgs::msg::Point checkpoint;
                        State aligningState;

                        {
                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                            changedState = stateData_.changedState;
                            checkpoint = stateData_.checkpoint;
                            aligningState = stateData_.aligningState;
                        }

                        if(changedState == true) {
                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                            stateData_.changedState = false;
                            
                            stateData_.aligningState = State::LEAVING;
                            stateData_.checkpoint = dronePose_.pose.pose.position;
                        }
                        else {

                            double midRowScaler;

                            {
                                std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                midRowScaler = cropData_.midRowScaler;
                            }

                            switch(aligningState)
                            {
                                // go forward X
                                case State::LEAVING: {

                                    double distance = sqrt(pow(dronePose_.pose.pose.position.x - checkpoint.x, 2) + pow(dronePose_.pose.pose.position.y - checkpoint.y, 2));

                                    // if distance is smaller than mid row scaler
                                    if(distance < midRowScaler) {

                                        // move forward
                                        geometry_msgs::msg::Twist vel;
                                        vel.linear.x = manualNavData_.linear;
                                        cmdVelPub_->publish(vel);
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.aligningState = State::TURNING_PERP;
                                        stateData_.checkpoint = dronePose_.pose.pose.position;
                                    }

                                    }

                                    break;

                                // turn 90 degrees to the right or left to face down field side
                                case State::TURNING_PERP: {
                                    // compute angle bearing
                                    double angleDesired;
                                    {
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        angleDesired = atan2(cropData_.rowPerpendicular.y, cropData_.rowPerpendicular.x);
                                    }
                                    angleDesired = correctAngle(angleDesired);
                                    double angle = angleDesired - dronePose_.pose.pose.orientation.z;
                                    angle = correctAngle(angle);
                                    int direction = angle / abs(angle);

                                    // check if drone is facing the right direction
                                    if(angle > 0.1) {
                                        geometry_msgs::msg::Twist vel;
                                        vel.angular.z = direction * manualNavData_.rotate;
                                        cmdVelPub_->publish(vel);
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.aligningState = State::A_MOVING;
                                        stateData_.checkpoint = dronePose_.pose.pose.position;
                                    }

                                    }

                                    break;

                                // go forward X*2
                                case State::A_MOVING: {
                                    
                                    double distance = sqrt(pow(dronePose_.pose.pose.position.x - checkpoint.x, 2) + pow(dronePose_.pose.pose.position.y - checkpoint.y, 2));

                                    // if distance is smaller than mid row scaler * 1.5
                                    if(distance < midRowScaler * 2) {

                                        // move forward
                                        geometry_msgs::msg::Twist vel;
                                        vel.linear.x = manualNavData_.linear;
                                        cmdVelPub_->publish(vel);
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.aligningState = State::TURNING_PARA;
                                        stateData_.checkpoint = dronePose_.pose.pose.position;
                                    }

                                    }

                                    break;

                                // turn 90 degrees to the right or left to face down row
                                case State::TURNING_PARA: {

                                    // set angle
                                    bool leftTurnRow;
                                    {
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        leftTurnRow = cropData_.leftTurnRow;
                                    }

                                    int direction;

                                    if(leftTurnRow == true) {
                                        direction = -1;
                                    }
                                    else {
                                        direction = 1;
                                    }
                                
                                    // compute angle bearing
                                    double angleDesired;
                                    {
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        angleDesired = direction * atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                                    }
                                    angleDesired = correctAngle(angleDesired);
                                    double angle = angleDesired - dronePose_.pose.pose.orientation.z;
                                    angle = correctAngle(angle);

                                    // check if drone is facing the right direction
                                    if(angle > 0.1) {

                                        // turn
                                        geometry_msgs::msg::Twist vel;
                                        vel.angular.z = direction * manualNavData_.rotate;
                                        cmdVelPub_->publish(vel);
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.aligningState = State::ENTERING;
                                        stateData_.checkpoint = dronePose_.pose.pose.position;
                                    }
                                
                                    }

                                    break;

                                // go forward X*1.5
                                case State::ENTERING: {

                                    double distance = sqrt(pow(dronePose_.pose.pose.position.x - checkpoint.x, 2) + pow(dronePose_.pose.pose.position.y - checkpoint.y, 2));

                                    // if distance is smaller than mid row scaler * 1.5
                                    if(distance < midRowScaler * 1.5) {

                                        // move forward
                                        geometry_msgs::msg::Twist vel;
                                        vel.linear.x = manualNavData_.linear;
                                        cmdVelPub_->publish(vel);
                                    }
                                    else {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.aligningState = State::CHECKING;
                                        stateData_.checkpoint = dronePose_.pose.pose.position;
                                    }

                                    }

                                    break;
                                
                                // check if row exists
                                case State::CHECKING: {

                                    // check if drone is facing down a row
                                    double angle;
                                    {
                                        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                        angle = atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                                    }
                                    angle = correctAngle(angle);

                                    if(dronePose_.pose.pose.orientation.z - angle > 0.1) {

                                        // check there is a row on either side of the drone using ground lidar
                                        std::vector<geometry_msgs::msg::Point> crops = determineRows(processLiDAR(copyLiDAR()));

                                        // if there is a row on either side of the drone
                                        if(crops.size() == 2) {
                                            // we are in a row move to sampling
                                            {
                                                std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                                                
                                                stateData_.superState = State::SAMPLING;
                                                stateData_.changedState = true;
                                            }
                                            
                                            {
                                                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                                                feedbackData_.state = "SAMPLING";
                                            }
                                        }
                                        else if(crops.size() == 1) {
                                            // we are done in this field switch to idle
                                            {
                                                std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                                stateData_.superState = State::IDLE;                    
                                                stateData_.changedState = true;
                                                stateData_.navDone = true;
                                            }

                                            {
                                                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                                                feedbackData_.state = "IDLE";
                                            }
                                        }
                                        else {
                                            // we are not in a row and there has been an issue switch to emergency
                                            {
                                                std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                                stateData_.superState = State::EMERGENCY;
                                                stateData_.changedState = true;
                                            }

                                            {
                                                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                                                feedbackData_.state = "EMERGENCY";
                                            }
                                        }
                                        
                                    }
                                    else {
                                        // set angle
                                        bool leftTurnRow;
                                        {
                                            std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                            leftTurnRow = cropData_.leftTurnRow;
                                        }
                                        
                                        int direction;

                                        if(leftTurnRow == true) {
                                            direction = -1;
                                        }
                                        else {
                                            direction = 1;
                                        }
                                    
                                        // compute angle bearing
                                        double angleDesired;
                                        {
                                            std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                            angleDesired = direction * atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                                        }
                                        angleDesired = correctAngle(angleDesired);
                                        double angle = angleDesired - dronePose_.pose.pose.orientation.z;
                                        angle = correctAngle(angle);
                                    
                                        geometry_msgs::msg::Twist vel;
                                        vel.angular.z = direction * manualNavData_.rotate;
                                        cmdVelPub_->publish(vel);
                                    }

                                    }

                                    break;

                                // default
                                default:
                                    
                                    break;
                            }
                        }

                        }

                        break;

                    // sampling
                    case State::SAMPLING: {

                        bool changedState;

                        {
                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                            changedState = stateData_.changedState;
                        }

                        // if just changed state
                        if(changedState == true) {

                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                            
                            stateData_.changedState = false;
                            
                            querySoil();

                        }
                        else {
                            geometry_msgs::msg::Twist vel;

                            // drone allignment check and lidar processing
                            double angle;
                            {
                                std::lock_guard<std::mutex> lock(cropData_.cropMutex);

                                angle = atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                            }
                            angle = correctAngle(angle);
                            int direction = angle / abs(angle);

                            // process lidar
                            std::vector<geometry_msgs::msg::Point> rows = determineRows(processLiDAR(copyLiDAR()));

                            // check still in row
                            if(rows.size() < 2) {

                                {
                                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                    stateData_.superState = State::ALIGNING;
                                    stateData_.changedState = true;
                                }

                                {
                                    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                                    feedbackData_.state = "ALIGNING";
                                }
                            }
                            else {
                                double distLeft = sqrt(pow(rows.at(0).x - dronePose_.pose.pose.position.x, 2) + pow(rows.at(0).y - dronePose_.pose.pose.position.y, 2));
                                double distRight = sqrt(pow(rows.at(1).x - dronePose_.pose.pose.position.x, 2) + pow(rows.at(1).y - dronePose_.pose.pose.position.y, 2));

                                double diff = distLeft - distRight;
                            
                                // check facing direction of drone
                                if(dronePose_.pose.pose.orientation.z - angle > 0.1) {
                                    vel.angular.z = direction * manualNavData_.rotate;
                                }
                                // check robot has not drifted too close to one side of the row
                                else if (abs(diff) > 0.1) {
                                    
                                    // check direction needed and set velocity
                                    if(diff > 0) {
                                        vel.linear.x = manualNavData_.rotate;
                                    }
                                    else {
                                        vel.linear.x = -1 * manualNavData_.rotate;
                                    }
                                }

                                // check if drone is in range of an obstacle switch to emergency
                                if(isObstacleInRange(dronePose_.pose.pose.position) == true) {
                                    vel.linear.x = 0;

                                    {
                                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                                        stateData_.superState = State::EMERGENCY;
                                        stateData_.changedState = true;
                                    }
                                    
                                    {
                                        std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);
                                    
                                        feedbackData_.state = "EMERGENCY";
                                    }

                                }
                                else {
                                    // check distance from last sample spot
                                    double distance = sqrt(pow(dronePose_.pose.pose.position.x - soil_.soilPose.pose.pose.position.x, 2) + pow(dronePose_.pose.pose.position.y - soil_.soilPose.pose.pose.position.y, 2));

                                    // if distance greater than X
                                    if(distance > minSampleDistance_) {
                                        
                                        querySoil();
                                    
                                    }
                                    else {
                                        // drive forward
                                        vel.linear.x = manualNavData_.linear;
                                    } 
                                }
        
                            }

                        }

                        }

                        break;

                    // emergency
                    case State::EMERGENCY: {

                        bool changedState;

                        {
                            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                            changedState = stateData_.changedState;
                        }

                        if(changedState == true) {

                            {
                                std::lock_guard<std::mutex> lock(stateData_.stateMutex);
                            
                                stateData_.changedState = false;
                            }
                            
                            {
                                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);
                            
                                feedbackData_.emergency = true;
                            }
                        
                        }

                        }

                        break;

                    // default
                    default:

                        break;
                }
            }
            else {

                bool changedState;

                // lock state mutex
                {
                    std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                    changedState = stateData_.changedState;
                }

                if(changedState == true) {

                    geometry_msgs::msg::Pose goal;

                    // publish to nav2 to travel to point
                    {
                        std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                        goal.position = stateData_.emergencyPosition;
                        stateData_.changedState = false;
                    }

                    publishNavGoals(goal);
                }
            }
        
            // clock loop end
            auto loop_end = robotClock::now();
            auto elapsed = loop_end - loop_start;
            auto sleep_time = loop_period_ - elapsed;

            // sleep if necessary
            if (sleep_time > std::chrono::nanoseconds(0)) {
                std::this_thread::sleep_for(sleep_time);
            }

            {
                std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                navDone = stateData_.navDone;
            }

            
        }
        
        // sleep thread for 500ms while its not doing anything (aka idle state)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        {
            std::lock_guard<std::mutex> lock(threadData_.threadMutex);

            threadExists = threadData_.threadExists;
        }

    }
// exit main loop
}


//-----------------  GETTERS AND SETTERS  ---------------//

// getter for odometry
nav_msgs::msg::Odometry Robot::RobotController::getOdometry()
{
    std::lock_guard<std::mutex> lock(positionData_.positionMutex);

    return positionData_.dronePose;
}


// getter for current goal
geometry_msgs::msg::Pose Robot::RobotController::getGoal()
{
    std::lock_guard<std::mutex> lock(goals_.goalsMutex);

    return goals_.currentGoal; //Set goal position to the currently active goal being navigated to
}


// setter for goals
void Robot::RobotController::setGoals(geometry_msgs::msg::PoseArray goals)
{
    // lock mutex and save goals
    std::lock_guard<std::mutex> lock(goals_.goalsMutex);
    
    goals_.rawGoals = goals;
    goals_.droneGoals = goals;

    if(goals_.rawGoals.poses.size() > 0) {

        // save crop field corner
        {
            std::lock_guard<std::mutex> lock(cropData_.cropMutex);

            cropData_.rowCorner = goals_.rawGoals.poses.back().position;
        }

        // lock thread mutex
        std::lock_guard<std::mutex> lock(threadData_.threadMutex);

        // start new nav thread and close previous nav thread if not finished
        if(threadData_.threadExists == false) {
            threadData_.navThread = new std::thread(&Robot::RobotController::navThread, this);
            threadData_.threadExists = true;
        }
        else {
            //lock state mutex and set state to travelling
            {
                std::lock_guard<std::mutex> lock(stateData_.stateMutex);

                stateData_.navDone = false;
                stateData_.superState = State::TRAVELLING;
                stateData_.changedState = true;
            }

            {
                std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                feedbackData_.state = "TRAVELLING";
            }
        }
    }
}


// getter for emergency
bool Robot::RobotController::getEmergency()
{
    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

    return feedbackData_.emergency;
}


// getter for status
std::string Robot::RobotController::getStatus()
{
    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

    return feedbackData_.state;
}


// getter for battery
int Robot::RobotController::getBattery()
{
    std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

    return feedbackData_.battery;
}


//-------------------  OTHER FUNCTIONS  -----------------//


// correct angle
double Robot::RobotController::correctAngle(double angle)
{
    if(angle > M_PI) {
        angle = angle - 2 * M_PI;
    }
    else if(angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }

    return angle;
}


// find 2 closest points on each object
std::vector<geometry_msgs::msg::Point> Robot::RobotController::twoClosest(std::vector<geometry_msgs::msg::Point> closestPoints)
{
    nav_msgs::msg::Odometry dronePose = getOdometry();

    // save closest 2 points
    geometry_msgs::msg::Point closestPoint = closestPoints.at(0);
    double closestDist = sqrt(pow(closestPoint.x - dronePose.pose.pose.position.x, 2) + pow(closestPoint.y - dronePose.pose.pose.position.y, 2));
    geometry_msgs::msg::Point nextClosestPoint = closestPoint;
    double nextClostestDist = closestDist;
    int pointTracker = 0;

    for(auto point : closestPoints) {
        if(pointTracker != 0) {

            // check if new closest point
            double dist = sqrt(pow(point.x - dronePose.pose.pose.position.x, 2) + pow(point.y - dronePose.pose.pose.position.y, 2));

            if(dist < closestDist) {    // if new closest point
                nextClosestPoint = closestPoint;
                nextClostestDist = closestDist;

                closestPoint = point;
                closestDist = dist;
            }
            else if(pointTracker == 1) {    // if second point but not closest
                nextClosestPoint = point;
                nextClostestDist = dist;
            }
            else if(dist < nextClostestDist) {  // if new second closest points
                nextClosestPoint = point;
                nextClostestDist = dist;
            }
        }

        pointTracker++;
    }

    // put closest points into vector
    std::vector<geometry_msgs::msg::Point> twoCloesest;
    twoCloesest.push_back(closestPoint);
    twoCloesest.push_back(nextClosestPoint);

    return twoCloesest;
}


// calculate allignment point on 2 closest points
geometry_msgs::msg::Point Robot::RobotController::calculateAllignmentPoint(std::vector<geometry_msgs::msg::Point> closestPoints)
{
    // to be returned (point where the robot should move to)
    geometry_msgs::msg::Point allignmentPoint;

    // calculate centroid of crops and direction of crops using two closest points
    allignmentPoint.x = (closestPoints.at(0).x + closestPoints.at(1).x) / 2;
    allignmentPoint.y = (closestPoints.at(0).y + closestPoints.at(1).y) / 2;

    // return allignment point
    return allignmentPoint;
}


// make local copy of lidar data
sensor_msgs::msg::LaserScan Robot::RobotController::copyLiDAR()
{
    sensor_msgs::msg::LaserScan crops;

    // lock objects mutex
    {
        std::lock_guard<std::mutex> lock(groundLiDAR_.groundLiDARMutex);

        // pushback crop objects into vector
        crops = groundLiDAR_.data;
    }

    return crops;
}


// convert raw lidar data into global vector of points
std::vector<geometry_msgs::msg::Point> Robot::RobotController::processLiDAR(sensor_msgs::msg::LaserScan lidar)
{
    std::vector<geometry_msgs::msg::Point> globalCartesian;
    globalCartesian.resize(lidar.ranges.size());

    nav_msgs::msg::Odometry dronePose = getOdometry();

    // convert polar to global
    for(unsigned int i = 0; i < lidar.ranges.size(); i++) {

        geometry_msgs::msg::Point localCart;

        double theta = lidar.angle_min + lidar.angle_increment * i;

        // convert from polar to cartesian
        localCart.x = lidar.ranges.at(i) * cos(theta);
        localCart.y = lidar.ranges.at(i) * sin(theta);
        localCart.z = 0;

        // apply LiDAR tilt correction (45 degrees down on x axis)
        localCart.y = localCart.y * cos(M_PI/4);
        localCart.z = localCart.y * sin(-M_PI/4);

        // convert from lidar to robot position frame
        localCart.x = localCart.x - groundLiDAR_.offset.x;
        localCart.y = localCart.y - groundLiDAR_.offset.y;
        localCart.z = localCart.z - groundLiDAR_.offset.z;

        // convert from robot to global frame
        localCart.x = localCart.x + dronePose.pose.pose.position.x;
        localCart.y = localCart.y + dronePose.pose.pose.position.y;
        localCart.z = localCart.z + dronePose.pose.pose.position.z;

        // add to vector
        globalCartesian.at(i) = localCart;
    }

    return globalCartesian;

}


// determine number of rows adjacent to the drone
std::vector<geometry_msgs::msg::Point> Robot::RobotController::determineRows(std::vector<geometry_msgs::msg::Point> lidar)
{
    std::vector<geometry_msgs::msg::Point> rows;

    bool newRow = true;
    double maxZ = 0;
    int maxIndex = 0;

    double midRowScaler;
    double bumpThreshold;
    {
        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

        midRowScaler = cropData_.midRowScaler;
        bumpThreshold = cropData_.bumpThreshold;
    }

    nav_msgs::msg::Odometry dronePose = getOdometry();

    // check number of bumps greater than bum threshold
    for(unsigned int i = 0; i < lidar.size(); i++) {
        
        // check is within row threshold (1.5* midRowScaler length)
        double distance = sqrt(pow(lidar.at(i).x - dronePose.pose.pose.position.x, 2) + pow(lidar.at(i).y - dronePose.pose.pose.position.y, 2));
        
        // check it is an adjacent row

        if(distance > midRowScaler * 1.5) {

            // check it is a genuine bump
            if(lidar.at(i).z > bumpThreshold) {

                // check if new row
                if(newRow == true) {

                    // set new max
                    maxZ = lidar.at(i).z;
                    maxIndex = i;
                    newRow = false;
                }
                else {

                    // check if new max
                    if(maxZ < lidar.at(i).z) {
                        maxZ = lidar.at(i).z;
                        maxIndex = i;
                    }
                }
            }
            else {
                newRow = true;
                rows.push_back(lidar.at(maxIndex));
            }
        }
    }

    return rows;

}


// identify initial crop field direction vectors
// determine number of rows adjacent to the drone
std::vector<geometry_msgs::msg::Point> Robot::RobotController::locateInitialRows(std::vector<geometry_msgs::msg::Point> lidar)
{
    std::vector<geometry_msgs::msg::Point> rows;

    bool newRow = true;
    double maxZ = 0;
    int maxIndex = 0;

    double bumpThreshold;
    {
        std::lock_guard<std::mutex> lock(cropData_.cropMutex);

        bumpThreshold = cropData_.bumpThreshold;
    }

    // check number of bumps greater than bum threshold
    for(unsigned int i = 0; i < lidar.size(); i++) {
    

        // check it is a genuine bump
        if(lidar.at(i).z > bumpThreshold) {

            // check if new row
            if(newRow == true) {

                // set new max
                maxZ = lidar.at(i).z;
                maxIndex = i;
                newRow = false;
            }
            else {

                // check if new max
                if(maxZ < lidar.at(i).z) {
                    maxZ = lidar.at(i).z;
                    maxIndex = i;
                }
            }
        }
        else {
            newRow = true;
            rows.push_back(lidar.at(maxIndex));
        }
    }

    return rows;

}


// check if any known obstacles are within range of the drone
bool Robot::RobotController::isObstacleInRange(geometry_msgs::msg::Point point)
{
    geometry_msgs::msg::PoseArray objects;
    double minRange;

    // lock obstacles mutex
    {
        std::lock_guard<std::mutex> lock(objects_.objectMutex);

        objects = objects_.objectPoints;
        minRange = objects_.minRange;
    }

        // check if any obstacles are within range of the drone
        for(auto obstacle : objects.poses) {
            if(sqrt(pow(point.x - obstacle.position.x, 2) + pow(point.y - obstacle.position.y, 2)) < minRange) {
                return true;
            }
        }

    return false;
}

// needs updating
// start main threaded loop
bool Robot::RobotController::autoNavigate()
{
    // lock goal mutex
    {
        std::lock_guard<std::mutex> lock(goals_.goalsMutex);

        // if goals raw exist
        if(goals_.rawGoals.poses.size() > 0) {

            std::lock_guard<std::mutex> lock(stateData_.stateMutex);

            //if nav thread is not running
            if(stateData_.navDone == false) {

                stateData_.navDone = true;

            }
            else {

                return false;
            }

        }
        else {

            return false;
        }
    
    }
    
    return true;
}


// soil sampling function 
void Robot::RobotController::querySoil()
{
    if(registered_station_ != ""){

        {
            std::lock_guard<std::mutex> lock(soil_.soilMutex);
            
            soil_.soilPose = getOdometry();
            last_query_odom_ = soil_.soilPose;
        
        }

        auto x = last_query_odom_.pose.pose.position.x;
        auto y = last_query_odom_.pose.pose.position.y;
        while(!soil_query_client_->wait_for_service(std::chrono::milliseconds(100))){
            RCLCPP_WARN(node_->get_logger(), "Waiting for soil query service to be available...");
        }
        auto request = std::make_shared<unomas::srv::QuerySoil::Request>();
        request->x = x;
        request->y = y;
        soil_query_client_->async_send_request(request,
            std::bind(&Robot::RobotController::soilRequestCallback, this, std::placeholders::_1)
        );
    }
}
