// includes
#include "path_planer.h"


//-------------------  CONSTRUCTOR AND DESTRUCTOR  -------------------//

// constructor
PathPlanner::PathPlanner(geometry_msgs::msg::PoseArray goals, geometry_msgs::msg::Pose currentPose)
{
    // initialise state
    state_ = WAITING;

    // get current pose of drone

    // initialise feedback data
    feedbackData_.emergency = false;
    feedbackData_.state = "waiting";
    feedbackData_.goal = [];

    // initialise feedback publishers
    statePub_ = this->create_publisher<std_msgs::msg::String>("/state", 10);
    emergencyPub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency", 10);
    goalPub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);

    // initialise publishers
    navPub_ = this->create_publisher<geometry_msgs/msg/PoseStamped>("/goal_pose", 10);
    soilPub_ = this->create_publisher<custom_msgs::SoilData>("soil", 10);
    cmdVelPub_ = this->create_publisher<geometry_msgs/msg/Twist>("/cmd_vel", 10);

    // initialise subscribers
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/goals", 10, std::bind(&PathPlanner::subscribeGoals, this, std::placeholders::_1));
    obstaclesSub_ = this->create_subscription<custom_msgs::Obstacles>("/obstacles", 10, std::bind(&PathPlanner::subscribeObstacles, this, std::placeholders::_1));
    groundLiDARSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/laserscan2", 10, std::bind(&PathPlanner::subscribeGroundLiDAR, this, std::placeholders::_1));


    // initialise wall timer for feedback pubs
    Timer_ = this->create_wall_timer(500ms, std::bind(&PathPlanner::timer_callback, this));
}


// destructor
PathPlanner::~PathPlanner()
{
    // close nav thread
    threadData_.navDone = true;
    threadData_.navThread.join();
    delete navThread_;
}


//-------------------  FEEDBACK PUBLISHERS  -------------------//

// publisher callback that publishes the current state of the robot to the base station
PathPlanner::publishState(std::string state)
{
    // make message
    std_msgs::msg::String msg;

    // lock feedback mutex
    {
        std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

        msg.data = feedBackData_.state;
    }

    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.data.c_str());

    // publish message
    statePub_->publish(msg);


}


// publisher callback that publishes the emergency state of the robot to the base station
PathPlanner::publishEmergency()
{
    // make message
    std_msgs::msg::Bool msg;

    // lock feedback mutex
    {
        std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

        msg.data = feedBackData_.emergency;
    }

    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.data.c_str());

    // publish message
    emergencyPub_->publish(msg);
}


// publisher callback that publishes the current goal of the robot to the base station
PathPlanner::publishGoal()
{
    // make message
    geometry_msgs::msg::PoseStamped msg;

    // lock feedback mutex
    {
        std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

        msg.pose = feedBackData_.goal;
    }

    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.pose.c_str());

    // publish message
    goalPub_->publish(msg);
}


// timer callback that calls feedback publishers
PathPlanner::timer_callback()
{
    // publish state
    publishState();

    // publish emergency
    publishEmergency();

    // publish goal
    publishGoal();
}

//-------------------  FUNCTIONAL SUBSCRIBERS  -------------------//

// subscriber callback that listens for goal pose instruction from base station
PathPlanner::subscribeGoals(geometry_msgs::msg::PoseArray goals)
{
    // lock mutex and save goals
    {
    std::lock_guard<std::mutex> lock(goalsData_.goalsMutex);
    
    goalsData_.rawGoals = goals;
    goalsData_.droneGoals = goals;
    }


    if(goalsData_.rawGoals.size() > 0) {

        // save crop field corner
        cropData_.rowCorner = goalsData_.rawGoals.back().position;

        // start new nav thread and close previous nav thread if not finished
        if(threadData_.threadExists == false) {
            threadData_.navThread = std::thread(&PathPlanner::navThread, this);
            threadData_.threadExists = true;
        }
        else {
            threadData_.navDone = true;
            threadData_.threadExists = false;
            threadData_.navThread.join();
            delete navThread_;

            threadData_.navThread = std::thread(&PathPlanner::navThread, this);
            threadData_.threadExists = true;
            threadData_.navDone = false;
        }
    }

    // publish when nav thread is done and delete nav thread?

}

// This whole subscriber needs fixing
// subscriber callback that listens for obstacles from detection node
PathPlanner::subscribeObstacles(sensor_msgs::msg::LaserScan data)
{
    // lock mutex and save data
    {
    std::lock_guard<std::mutex> lock(objects_.objectMutex);
    
    objects_.objectPoints = data.ranges;
    }
}


// subscriber callback that listens for ground LiDAR
PathPlanner::subscribeGroundLiDAR(sensor_msgs::msg::LaserScan laser)
{
    // lock mutex and save laser data
    {
    std::lock_guard<std::mutex> lock(groundLiDAR_.groundLiDARMutex);
    
    groundLiDAR_.data = laser;
    }
}


//-------------------  FUNCTIONAL PUBLISHERS  -------------------//

// publisher callback that publishes soil data
PathPlanner::publishSoil()
{
    // make message
    custom_msgs::SoilData msg;

    // lock soil mutex
    {
        std::lock_guard<std::mutex> lock(soilData_.soilMutex);

        msg.soilPose = feedBackData_.soilPose;
        msg.soilData = feedBackData_.soilData;
    }

    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.soilData.c_str());

    // publish message
    soilPub_->publish(msg);
}


// publisher callback that publishes velocity commands
PathPlanner::publishCmdVel(geometry_msgs::msg::Twist vel)
{
    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", vel.c_str())

    // publish message
    cmdVelPub_->publish(vel);

}


// publisher callback that publishes goal poses to Nav2
PathPlanner::publishGoals()
{
    // make message
    geometry_msgs::msg::PoseStamped msg;

    // lock goals mutex
    {
        std::lock_guard<std::mutex> lock(goalsData_.goalsMutex);

        msg.pose = goals_.currentGoal;
    }

    // log message
    RCLCPP_INFO(this->get_logger(), "Publishing %s", msg.pose.c_str());

    // publish message
    navPub_->publish(msg);
}

//-------------------  MAIN THREADED LOOP  -----------------//


// main threaded loop that runs the planner (4 states: travelling, aligning, sampling, emergency)
PathPlanner::navThread()
{

    stateData_.superState = TRAVELLING;
    feedbackData_.state = "TRAVELLING";

    while(threadData_.navDone == false)
    {

        // get odo service call to base station
        // ... (ask mattia for advice) ...

        // do state actions
        switch(stateData_.superState)
        {
            // travelling to field guess
            case TRAVELLING:

                // publish to nav2 and moniter status
                if(goals_.droneGoals.size() > 0) {

                    // check if drone is already moving to a goal
                    if(goals_.commuting == false) {
                        // set goal
                        goals_.currentGoal = goals_.droneGoals.at(0);
                        goals_.commuting = true;
                        publishGoal();
                    }
                    else {

                        distance = sqrt(pow(goals_.currentGoal.position.x - robotPose_.position.x, 2) + pow(goals_.currentGoal.position.y - robotPose_.position.y, 2));

                        // check goal has been reached
                        if( distance < 0.05) {
                            // set new goal
                            goals_.droneGoals.erase(goals_.droneGoals.begin());
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
                                state_ = EMERGENCY;
                                feedbackData_.state = "EMERGENCY";
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
                    state_ = ALLIGNING;
                    feedbackData_.state = "ALLIGNING";
                }


                break;

            // surveying
            case SURVEYING:

                switch(stateData_.surveyingState)
                {
                    // do a rotation and identify closest (starting row/current row) and second closest (next row) row centre to beacon
                    case ROTATING:

                        // if first loop grab initial bearing
                        if(stateData_.changedState == true) {

                            stateData_.changedState = false;

                            stateData_.initSurveyingAngle = robotPose_.theta;
                            stateData_.startedRotating = false;
                        }
                        else {
                        
                            // save points in crop centres if they are not already saved
                            collectObjectPoints();

                            // rotate robot
                            geometry_msgs::msg::Twist vel;
                            vel.angular.z = manualNavData_.rotate;
                            cmdVelPub_->publish(vel);

                            // check if robot has rotated enough
                            if(robotPose_.theta - stateData_.initSurveyingAngle < 0.1) {
                                
                                // ensure it has started rotating
                                if(stateData_.startedRotating == true) {

                                    stateData_.surveyingState = CALCULATING;
                                }
                            }
                            else {
                                stateData_.startedRotating = true;
                            }
                        }

                        break;

                // calculate middle of row, calculate direction and distance to move to middle of row (i.e. direction of line, move half distance of line)
                    case CALCULATING:

                        // find closest 2 crop centres
                        std::vector<geometry_msgs::msg::Point> crops = twoClosest(cropsData_.cropCentres); //alternatively use this to make a local copy of the var

                        cropData_.currentCrop = crops.at(0);
                        cropData_.nextCrop = crops.at(1);

                        // find row centre
                        cropData_.rowCentre = calculateAllignmentPoint(crops);

                        // calculate crop field side direction vectors
                        cropData_.rowParallel = cropData_.currentCrop - dronePose_.position;
                        
                        magnitude = sqrt(pow(cropData_.rowParallel.x, 2) + pow(cropData_.rowParallel.y, 2));
                        cropData_.rowParallel.x = cropData_.rowParallel.x / magnitude;
                        cropData_.rowParallel.y = cropData_.rowParallel.y / magnitude;
                
                        cropData_.rowPerpendicular.x = -1 * cropData_.rowParallel.y;
                        cropData_.rowPerpendicular.y = cropData_.rowParallel.x;

                        // calculate projection of crop row centre onto crop field perpendicular direction vector
                        geometry_msgs::msg::Point rowCentreProj = cropData_.rowCentre - dronePose_.position;
                        cropData_.midRowScaler = rowCentreProj.x * cropData_.rowPerpendicular.x + rowCentreProj.y * cropData_.rowPerpendicular.y;
                        cropData_.midRowVector.x = cropData_.rowPerpendicular.x * cropData_.midRowScaler;
                        cropData_.midRowVector.y = cropData_.rowPerpendicular.y * cropData_.midRowScaler;

                        // change sub-state
                        stateData_.surveyingState = MOVING;

                        break;
                    
                    // move forward to row middle and turn to face down row (rotate 90 to the right)
                    case S_MOVING:
                        geometry_msgs::msg::Twist vel;

                        if(stateData_.rowAlligned == false) {
                            // calculate angle between rowPerpendicular and drone
                            angle = atan2(cropData_.rowPerpendicular.y, cropData_.rowPerpendicular.x);
                            angle = correctAngle(angle);
                            direction = angle / abs(angle);

                            // calculate magnitude of distance from corner to drone
                            distance = sqrt(pow(dronePose_.position.x - cropData_.rowCorner.x, 2) + pow(dronePose_.position.y - cropData_.rowCorner.y, 2));

                            // check if drone is facing perp to row and close to mid row
                            if(dronePose_.theta - angle > 0.1) {

                                vel.angular = direction * manualNavData_.rotate;
                            }
                            else if(distance - cropData_.midRowScaler > 0.1) {
                                vel.linear = manualNavData_.linear;
                            }
                            else {
                                stateData_.rowAlligned = true;
                            }
                        }
                        else {
                            // calculate angle between rowParallel and drone
                            angle = atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                            angle = correctAngle(angle);
                            direction = angle / abs(angle);

                            // check if drone is facing down row
                            if(dronePose_.theta - angle > 0.1) {
                                vel.angular = direction * manualNavData_.rotate;
                            }
                            else { 
                                stateData_.surveyingState = EXITING;
                            }
                        }

                        cmdVelPub_->publish(vel);

                        break;

                    // switch to sampling
                    case EXITING:
                        
                        // reset state data
                        stateData_.state = SAMPLING;
                        stateData_.surveyingState = ROTATING;
                        stateData_.changedState = true;
                        stateData_.rowAlligned = false;

                        break;
                }

                break;

            // aligning
            case ALIGNING:

                if(stateData_.changedState == true) {
                    stateData_.changedState = false;
                    
                    stateData_.aligningState = LEAVING;
                    stateData_.checkPoint = dronePose_.position;
                }
                else {
                    switch(stateData_.aligningState)
                    {
                        // go forward X
                        case LEAVING:

                            double distance = sqrt(pow(dronePose_.position.x - stateData_.checkpoint.x, 2) + pow(dronePose_.position.y - stateData_.checkpoint.y, 2));

                            // if distance is smaller than mid row scaler
                            if(distance < cropData_.midRowScaler) {

                                // move forward
                                geometry_msgs::msg::Twist vel;
                                vel.linear = manualNavData_.linear;
                                cmdVelPub_->publish(vel);
                            }
                            else {
                                stateData_.aligningState = TURNING_PERP;
                                stateData_.checkPoint = dronePose_.position;
                            }

                            break;
                        // turn 90 degrees to the right or left to face down field side
                        case TURNING_PERP:
                            // compute angle bearing
                            double angleDesired = atan2(cropData_.rowPerpendicular.y, cropData_.rowPerpendicular.x);
                            double angleDesired = correctAngle(angleDesired);
                            double angle = angleDesired - dronePose_.theta;
                            double angle = correctAngle(angle);
                            int direction = angle / abs(angle);

                            // check if drone is facing the right direction
                            if(angle > 0.1) {
                                geometry_msgs::msg::Twist vel;
                                vel.angular.z = direction * manualNavData_.rotate;
                                cmdVelPub_->publish(vel);
                            }
                            else {
                                stateData_.aligningState = A_MOVING;
                                stateData_.checkpoint = dronePose_.position;
                            }

                            break;

                        // go forward X*2
                        case A_MOVING:
                            
                            double distance = sqrt(pow(dronePose_.position.x - stateData_.checkpoint.x, 2) + pow(dronePose_.position.y - stateData_.checkpoint.y, 2));

                            // if distance is smaller than mid row scaler * 1.5
                            if(distance < cropData_.midRowScaler * 2) {

                                // move forward
                                geometry_msgs::msg::Twist vel;
                                vel.linear = manualNavData_.linear;
                                cmdVelPub_->publish(vel);
                            }
                            else {
                                stateData_.aligningState = TURNING_PARA;
                                stateData_.checkpoint = dronePose_.position;
                            }

                            break;

                        // turn 90 degrees to the right or left to face down row
                        case TURNING_PARA:

                            // set angle
                            if(cropData_.leftTurnRow == true) {
                                int direction = -1;
                            }
                            else {
                                int direction = 1;
                            }
                        
                            // compute angle bearing
                            double angleDesired = direction * atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                            double angleDesired = correctAngle(angleDesired);
                            double angle = angleDesired - dronePose_.theta;
                            double angle = correctAngle(angle);

                            // check if drone is facing the right direction
                            if(angle > 0.1) {

                                // turn
                                geometry_msgs::msg::Twist vel;
                                vel.angular.z = direction * manualNavData_.rotate;
                                cmdVelPub_->publish(vel);
                            }
                            else {
                                stateData_.aligningState = ENTERING;
                                stateData_.checkpoint = dronePose_.position;
                            }

                        // go forward X*1.5
                        case ENTERING:

                            double distance = sqrt(pow(dronePose_.position.x - stateData_.checkpoint.x, 2) + pow(dronePose_.position.y - stateData_.checkpoint.y, 2));

                            // if distance is smaller than mid row scaler * 1.5
                            if(distance < cropData_.midRowScaler * 1.5) {

                                // move forward
                                geometry_msgs::msg::Twist vel;
                                vel.linear = manualNavData_.linear;
                                cmdVelPub_->publish(vel);
                            }
                            else {
                                stateData_.aligningState = CHECKING;
                                stateData_.checkpoint = dronePose_.position;
                            }

                            break;
                        
                        // check if row exists
                        case CHECKING:

                            // check if drone is facing down a row
                            if(dronePose_.theta - angle > 0.1) {

                                // check there is a row on either side of the drone using ground lidar
                                int crops = determineRows(processLiDAR(copyLiDAR()));
                                
                                // if there is a row on either side of the drone
                                if(crops.size() == 2) {
                                    // we are in a row move to sampling
                                    stateData_.state = SAMPLING;
                                    stateData_.changedState = true;
                                }
                                else if(crops.size() == 1) {
                                    // we are done in this field switch to idle
                                    stateData_.state = IDLE;
                                    stateData_.changedState = true;
                                    navThreadDone = true;
                                }
                                else {
                                    // we are not in a row and there has been an issue switch to emergency
                                    stateData_.state = EMERGENCY;
                                    stateData_.changedState = true;
                                }
                                 
                            }
                            else {
                                // set angle
                                if(cropData_.leftTurnRow == true) {
                                    int direction = -1;
                                }
                                else {
                                    int direction = 1;
                                }
                            
                                // compute angle bearing
                                double angleDesired = direction * atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                                double angleDesired = correctAngle(angleDesired);
                                double angle = angleDesired - dronePose_.theta;
                                double angle = correctAngle(angle);
                            
                                geometry_msgs::msg::Twist vel;
                                vel.angular.z = direction * manualNavData_.rotate;
                                cmdVelPub_->publish(vel);
                            }


                            break;
                    }
                }

                break;

            // sampling
            case SAMPLING:

                //make local copy of LiDAR data

                // if just changed state
                if(stateData_.changedState == true) {
                    
                    stateData_.changedState = false;
                    
                    // sample function
                }
                else {
                    geometry_msgs::msg::Twist vel;

                    // drone allignment check and lidar processing
                    double angle = atan2(cropData_.rowParallel.y, cropData_.rowParallel.x);
                    angle = correctAngle(angle);
                    direction = angle / abs(angle);

                    // process lidar
                    rows = determineRows(processLiDAR(copyLiDAR()));

                    // check still in row
                    if(rows.size() < 2) {
                        stateData_.state = ALIGNING;
                        stateData_.changedState = true;
                        break;
                    }
                    else {
                        distLeft = sqrt(pow(rows.at(0).x - dronePose_.position.x, 2) + pow(rows.at(0).y - dronePose_.position.y, 2));
                        distRight = sqrt(pow(rows.at(1).x - dronePose_.position.x, 2) + pow(rows.at(1).y - dronePose_.position.y, 2));

                        diff = distLeft - distRight;
                    }

                    
                    // check facing direction of drone
                    if(dronePose_.theta - angle > 0.1) {
                        vel.angular.z = direction * manualNavData_.rotate;
                    }
                    // check robot has not drifted too close to one side of the row
                    else if (abs(diff) > 0.1) {
                        
                        // check direction needed and set velocity
                        if(diff > 0) {
                            vel.linear = manualNavData_.linear;
                        }
                        else {
                            vel.linear = -1 * manualNavData_.linear;
                        }
                    }

                    // check distance from last sample spot
                    double distance = sqrt(pow(dronePose_.position.x - soil_.soilPose.position.x, 2) + pow(dronePose_.position.y - soil_.soilPose.position.y, 2));

                    // if distance greater than X
                    if(distance > stateData_.minSampleDistance) {
                        
                        // sample function
                    
                    }
                    else {
                        // drive forward
                        vel.linear = manualNavData_.linear;
                    } 

                }

                break;

            // emergency
            case EMERGENCY:

                if(stateData_.changedState == true) {

                    stateData_.changedState = false;

                    // lock mutex for feedback data
                    {
                        std::lock_guard<std::mutex> lock(feedbackData_.feedbackMutex);

                        feedbackData_.emergency = true;
                    }
                
                }

                break;
        }
        
        // check if thread is done
    }   
// exit main loop
}

//-------------------  OTHER FUNCTIONS  -----------------//

// correct angle
double PathPlanner::correctAngle(double angle)
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
std::vector<geometry_msgs::msg::Point> PathPlanner::twoClosest(std::vector<geometry_msgs::msg::Point> closestPoints)
{
    // save closest 2 points
    geometry_msgs::msg::Point closestPoint = closestPoints.at(0);
    double closestDist = sqrt(pow(closestPoint.x - robotPose_.position.x, 2) + pow(closestPoint.y - robotPose_.position.y, 2));
    geometry_msgs::msg::Point nextClosestPoint = closestPoint;
    double nextClostestDist = closestDist;
    int pointTracker = 0;

    for(auto point : closestPoints) {
        if(pointTracker != 0) {

            // check if new closest point
            dist = sqrt(pow(point.x - robotPose_.position.x, 2) + pow(point.y - robotPose_.position.y, 2));
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
geometry_msgs::msg::Point PathPlanner::calculateAllignmentPoint(std::vector<geometry_msgs::msg::Point> closestPoints)
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
std::vector<geometry_msgs::msg::Point> PathPlanner::copyLiDAR()
{
    std::vector<geometry_msgs::msg::Point> crops;

    // lock objects mutex
    {
        std::lock_guard<std::mutex> lock(groundLiDAR_.groundLiDARMutex);

        // pushback crop objects into vector
        crops = groundLiDAR_.data.ranges;
    }

    return crops;
}


// make local copy of detected crop rows
std::vector<geometry_msgs::msg::Point> PathPlanner::copyObjects()
{
    std::vector<geometry_msgs::msg::Point> crops;

    // lock objects mutex
    {
        std::lock_guard<std::mutex> lock(objects_.groundLiDARMutex);

        // pushback crop objects into vector
        crops = objects_.groundLiDAR.data.ranges;
    }

    return crops;
}


// convert raw lidar data into global vector of points
std::vector<geometry_msgs::msg::Point> PathPlanner::processLiDAR(std::vector<geometry_msgs::msg::Point> lidar) 
{
    std::vector<geometry_msgs::msg::Point> globalCartesian;

    // convert polar to global
    for(int i = 0; i < lidar.size(); i++) {

        geometry_msgs::msg::Point localCart;

        theta = lidar.angle_min + lidar.angle_increment * i;

        // convert from polar to cartesian
        localCart.x = lidar.at(i) * cos(theta);
        localCart.y = lidar.at(i) * sin(theta);
        localCart.z = 0;

        // apply LiDAR tilt correction (45 degrees down on x axis)
        localCart.y = LocalCart.y * cos(M_PI/4);
        LocalCart.z = LocalCart.y * sin(-M_PI/4);

        // convert from lidar to robot position frame
        LocalCart.x = LocalCart.x - LidarOffset.x;
        LocalCart.y = LocalCart.y - LidarOffset.y;
        LocalCart.z = LocalCart.z - LidarOffset.z;

        // convert from robot to global frame
        LocalCart.x = LocalCart.x + dronePose_.position.x;
        LocalCart.y = LocalCart.y + dronePose_.position.y;
        LocalCart.z = LocalCart.z + dronePose_.position.z;

        // add to vector
        globalCartesian.at(i) = LocalCart;
    }

    return globalCartesian;

}


// determine number of rows adjacent to the drone
std::vector<geometry_msgs::msg::Point> PathPlanner::determineRows(std::vector<geometry_msgs::msg::Point> lidar)
{
    std::vector<geometry_msgs::msg::Point> rows;

    bool newRow = true;
    double maxZ = 0;
    double maxIndex = 0;

    // check number of bumps greater than bum threshold
    for(int i = 0; i < lidar.size(); i++) {
        
        // check is within row threshold (1.5* midRowScaler length)
        double distance = sqrt(pow(lidar.at(i).x - dronePose_.position.x, 2) + pow(lidar.at(i).y - dronePose_.position.y, 2));
        
        // check it is an adjacent row
        if(distance > cropData_.midRowScaler * 1.5) {

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


// identify the 2 closest 