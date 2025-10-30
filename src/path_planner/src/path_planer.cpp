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


// subscriber callback that listens for obstacles from detection node
PathPlanner::subscribeObstacles(sensor_msgs::msg::LaserScan data)
{
    // lock mutex and save data
    {
    std::lock_guard<std::mutex> lock(groundLiDAR_.groundLiDARMutex);
    
    groundLiDAR_.data = data;
    }
}


// subscriber callback that listens for ground LiDAR
PathPlanner::subscribeGroundLiDAR(sensor_msgs::msg::LaserScan laser)
{
    // lock mutex and save laser data
    {
    std::lock_guard<std::mutex> lock(objects_.objectMutex);
    
    objects_.groundLiDAR = laser;
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


//-------------------  OTHER FUNCTIONS AND THREADS -----------------//

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


// detect crops
std::vector<geometry_msgs::msg::Point> PathPlanner::detectCrops()
{
    std::vector<geometry_msgs::msg::Point> crops;

    // lock objects mutex
    {
        std::lock_guard<std::mutex> lock(objects_.objectMutex);

        // pushback crop objects into vector
        for (auto object : objects_.objectPoints)
        {
            if (object.name == "crop")
            {
                crops.push_back(object.point);
            }   
        }
    }

    return crops;
}


// determine distinct objects from an array of points
std::vector<std::vector<geometry_msgs::msg::Point>> PathPlanner::determineObjects(std::vector<geometry_msgs::msg::Point> crops) 
{
    // determine seperate objects
    std::vector<std::vector<geometry_msgs::msg::Point>> objects;    // each vector contains all points of one object
    objects.push_back(std::vector<geometry_msgs::msg::Point>)
    objects.at(0).push_back(crops.at(0));    // start by putting first point into first object
    int lastObject = 0;     // to say which object the last point belongs to
    int pointsLogged = 1;   // number of points logged in current object

    for(int i = 1; i < crops.size(); i++) {

        // dist between current point and last point in current object
        float dist = sqrt(pow(crops.at(i).x - objects.at(lastObject).at(pointsLogged-1).x, 2) + pow(crops.at(i).y - objects.at(lastObject).at(pointsLogged-1).y, 2));
        
        if(dist > minDistObjects) {
            // if distance is larger than minDistObjects, then add point to new object
            objects.push_back(std::vector<geometry_msgs::msg::Point>());
            objects.at(lastObject+1).push_back(crops.at(i));
            lastObject++;
            pointsLogged = 1;
        }
        else {
            // if distance is smaller than minDistObjects, then add point to current object
            objects.at(lastObject).push_back(crops.at(i));
            pointsLogged++;
        }
    }

    return objects;
}

std::vector<geometry_msgs::msg::Point> PathPlanner::processLiDAR()
{
    // ...
}

// find the closest point on each object
std::vector<geometry_msgs::msg::Point> PathPlanner::findClosestPoints(std::vector<std::vector<geometry_msgs::msg::Point>> objects)
{

    std::vector<geometry_msgs::msg::Point> closestPoints;

    // calculate the closest crop from each object and keep the closest 2
    if(objects.size() < 1) {
        RCLCPP_ERROR(this->get_logger(), "Error: No objects detected");
        return closestPoints;
    }

    int pointTracker = 0;
    int objectTracker = 0;

    // check for smallest distance per object
    for(auto object : objects) {
        for(auto point : object) {
            if( pointTracker == 0 ) {
                closestPoints.push_back(point);
            }
            else {
                float dist1 = sqrt(pow(point.x - robotPose_.position.x, 2) + pow(point.y - robotPose_.position.y, 2));
                float dist2 = sqrt(pow(closestPoints.at(objectTracker).x - robotPose_.position.x, 2) + pow(closestPoints.at(objectTracker).y - robotPose_.position.y, 2));

                if(dist1 < dist2) {
                    closestPoints.at(objectTracker) = point;
                }
            }

            pointTracker++;
        }

        objectTracker++;
        pointTracker = 0;
    }

    return closestPoints;
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


// get raw object data
std::vector<geometry_msgs::msg::Point> PathPlanner::getRawObjectData()
{
    std::vector<geometry_msgs::msg::Point> rawObjects;

    // lock objects mutex
    {
        std::lock_guard<std::mutex> lock(objects_.objectMutex);

        rawObjects = objects_.objectPoints;
    }

    return rawObjects;

}

// Get goal data
std::vector<geometry_msgs::msg::Pose> PathPlanner::getGoalData()
{
    std::vector<geometry_msgs::msg::Pose> goalData;
    
    // lock goals mutex
    {
        std::lock_guard<std::mutex> lock(goalsData_.goalsMutex);
        
        goalData = goals_.rawGoals;
    }

    return goalData;
}


// main threaded loop that runs the planner (4 states: travelling, aligning, sampling, emergency)
PathPlanner::navThread()
{

    state_ = TRAVELLING;
    feedbackData_.state = "TRAVELLING";

    while(threadData_.navDone == false)
    {

        // get odo service call to base station
        // ... (ask mattia for advice) ...


        // do state actions
        switch(state_)
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





                // move forward to row middle and turn to face down row (rotate 90 to the right)

                // switch to sampling

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
                        double projScaler = rowCentreProj.x * cropData_.rowPerpendicular.x + rowCentreProj.y * cropData_.rowPerpendicular.y;
                        cropData_.midRowVector.x = cropData_.rowPerpendicular.x * projScaler;
                        cropData_.midRowVector.y = cropData_.rowPerpendicular.y * projScaler;

                        // change sub-state
                        stateData_.surveyingState = MOVING;

                        break;

                    case MOVING:

                        if(rowAlligned == false) {
                            // face rowPerpendicular

                        break;

                    case EXITING:

                        break;
                }

                break;


            // aligning
            case ALIGNING:

                // save current row into "past rows" array, save "next row" into "current row"

                // ID all row centres, remove row centres near already completed rows, find row centre closest to "current row" and save as next row

                // find midpoint between "current row" and "next row", calculate direction and distance to move to midpoint
                
                // turn to direction of line, move forward, then turn 90 degrees to face down row (maybe a bool that gets toggled between left and right? (or -1 and 1))

                // switch to sampling





                // detect obstacles to detect the field rows
                std::vector<geometry_msgs::msg::Point> crops = detectCrops(); //alternatively use this to make a local copy of the var

                // calculate allginment position
                std::vector<std::vector<geometry_msgs::msg::Point>> objects = determineObjects(crops);
                std::vector<geometry_msgs::msg::Point> closestPoints = findClosestPoints(objects);
                std::vector<geometry_msgs::msg::Point> twoClosestPoints = twoClosest(closestPoints);
                geometry_msgs::msg::Point allignmentPoint = calculateAllignmentPoint(twoClosestPoints);

                // check that allignment point is not in already sampled row
                for(auto point : sampledPoints) {

                    distance = sqrt(pow(point.x - allignmentPoint.x, 2) + pow(point.y - allignmentPoint.y, 2));
                    if(distance < minSampleDistance) {
                        RCLCPP_ERROR(this->get_logger(), "Error: Allignment point is in already sampled row");
                        
                        // if allignment point is in already sampled row, then move to sampling
                    }
                }

                // OPTION 1: Manual allignment
                    // determine velocity to move to allignment position
                    // publish to cmd_vel and moniter status
                        // if cannot allign after X time move to emergency
                
                // OPTION 2: Automatic allignment
                    // Publish to nav2 and moniter status
                        // if Nav2 is stuck and not moving, then cancel and republish goal
                            // if cannot reach goal after X time move to emergency
                    
                // if allignment is complete, then move to sampling


                break;

            // sampling
            case SAMPLING:

                // determine if still in field by using the ground pointing LiDAR to detect the field rows (convert ground facing LiDAR points to see elevation in ground over a certian treshold)

                // if still in field, then check if outside of sample range

                    // if outside of sample range, then sample

                    // else 
                        // drive forward
                
                // else
                    // drive forward and do a 180 rotation towards the field

                    // move to allgining





                if(stateData_.endSampling_ == false) {

                    // find 2 closest points on each object
                    std::vector<geometry_msgs::msg::Point> rawPoints = getRawObjectData(); //alternatively use this to make a local copy of the var
                    std::vector<std::vector<geometry_msgs::msg::Point>> objects = determineObjects(rawPoints);
                    std::vector<geometry_msgs::msg::Point> crops = findClosestPoints(objects);
                    std::vector<geometry_msgs::msg::Point> twoClosestPoints = twoClosest(crops);

                    // determine perpendicular allignment
                    double paraAngle = atan2(twoClosestPoints.at(1).y - twoClosestPoints.at(0).y, twoClosestPoints.at(1).x - twoClosestPoints.at(0).x);
                    double perpAngle = paraAngle + M_PI / 2;

                    // calculate rotation needed to align with field
                    double rotation = perpAngle - robotPose_.theta;
                    rotation = correctAngle(rotation);

                    // check parallel to field
                    if(rotation > 0.1) {

                        rotVel = rotation * vel_.rotate;

                        // allign with field
                        publishCmdVel(vel_.rotate);
                    }
                    else {
                        // begin probing process
                        bool sampleHere = true;

                        for( point : sampledPoints) {

                            distance = sqrt(pow(point.x - robotPose_.position.x, 2) + pow(point.y - robotPose_.position.y, 2));

                            if(distance > minSampleDistance) {
                                // take sample
                                sampleHere = false;
                            }
                        }

                        if(sampleHere == true) {
                            // sample location
                            // ... service call to get sample ...
                            // ... publish to soilData ...
                        }
                        else {
                            // move forward slightly
                            // ... publish to cmd_vel ...
                        }
                    }

                    // determine if still in field
                    std::vector<geometry_msgs::msg::Point> crops = detectCrops(); //alternatively use this to make a local copy of the var
                    if(crops.size() == 0) {
                        // count up and if counter reaches 10 then move forward  and rotate 180 degrees
                        stateData_.noCropsCounter++;
                        if(stateData_.noCropsCounter > 10) {
                            stateData_.noCropsCounter = 0;
                            stateData_.endSampling = true;
                            stateData_.moveTimer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                        }
                        // then switch to alligning
                    }
                    else {
                        stateData_.noCropsCounter = 0;
                    }
                }
                else {
                    if(stateData_.exitMove == true) {
                        // publish to cmd_vel to move forward
                        // ... publish to cmd_vel ...
                        if(stateData_.moveTimer > 2000) {
                            stateData_.exitMove = false;
                            double preSpin = robotPose_.theta;
                        }
                    }
                    else {
                        // publish to cmd_vel to rotate 180 degrees
                        // ... publish to cmd_vel ...
                        
                        // calculate whether robot has turned 180 
                        if(correctAngle(robotPose_.theta - M_PI) - preSpin < 0.1) {
                            // change state
                            stateData_.endSampling = false;
                        }
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