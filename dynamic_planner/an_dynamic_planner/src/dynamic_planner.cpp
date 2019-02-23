#include "an_dynamic_planner/dynamic_planner.h"

void Planner::getPoseCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_DEBUG("[Planner] Entering pose callback");
    start_ = *msg;
    pose_received_ = true;
}

void Planner::getGoalCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_DEBUG("[Planner] Entering goal callback");
    goal_ = *msg;
    goal_received_ = true;
}

void Planner::getObstaclesCallback (const an_messages::obstacles::ConstPtr& msg) {
    ROS_DEBUG_ONCE("[Planner] Entering obstacles callback");
    obstacles_ = *msg;
    obstacle_received_ = true;
}

void Planner::getMapCallback (const an_messages::lanes::ConstPtr& msg) {
    ROS_DEBUG_ONCE("[Planner] Entering Map callback");
    int num_lanes = msg->lanes.size();
    int num_stations_per_lane = static_cast<int>((msg->lanes[0].centerline[1].x
            - msg->lanes[0].centerline[0].x) / kStationLength) + 1;
    map_ = std::vector<std::vector<bool>> (num_lanes, std::vector<bool>(num_stations_per_lane,  false));
    map_received_ = true;
}

// loads pre-calculated motion primitives
bool Planner::loadMprim() {
    std::string mprim_file;
    ros::param::get("/MPRIM_FILE", mprim_file);
    std::ifstream ifs(mprim_file);
    if (!ifs) {
        ROS_WARN("[Motion Primitives] No Motion Primitve File");
        ifs.close();
        return false;
    }
    int num_motions;
    ifs >> num_motions;
    motions_.resize(num_motions);
    for (int i = 0; i < num_motions; ++i) {
        for (int j = 0; j < 4; ++j) {
            ifs >> motions_[i].start[j];
        }
        for (int j = 0; j < 4; ++j) {
            ifs >> motions_[i].end[j];
        }
        ifs >> motions_[i].length >> motions_[i].cost >> motions_[i].num_entries;
        motions_[i].trajectory.resize(motions_[i].num_entries);
        for (int j = 0; j < motions_[i].num_entries; ++j) {
            ifs >> motions_[i].trajectory[j].x;
            ifs >> motions_[i].trajectory[j].y;
            ifs >> motions_[i].trajectory[j].theta;
            ifs >> motions_[i].trajectory[j].time;
            ifs >> motions_[i].trajectory[j].velocity;
        }
    }
    ifs.close();
    ROS_DEBUG("[Motion Primitives] Motion primitives loaded successfully");
    return true;
}

//initialization
bool Planner::init() {
    //initialize callback flags
    new_plan_ = true;
    pose_received_ = false;
    goal_received_ = false;
    obstacle_received_ = false;
    map_received_ = false;
    //load motion primitive
    if (!loadMprim()) return false;
    //set up subscribers and publishers
    sub_pose_ = nh_.subscribe("pose", 5, &Planner::getPoseCallback, this);
    sub_goal_ = nh_.subscribe("goal", 5, &Planner::getGoalCallback, this);
    sub_obstacles_ = nh_.subscribe("obstacles", 5, &Planner::getObstaclesCallback, this);
    sub_map_ = nh_.subscribe("lanes", 1, &Planner::getMapCallback, this);
    pub_trajectory_ = nh_.advertise<an_messages::trajectory>("planner_trajectory", 1, true);
    //set loop rate [Hz]
    loop_rate_ = 100;
    return true;
}

//calculate heuristic value
double Planner::heuristic(int lane, int station) {
    return 100.0 * abs(goal_.pose.position.y / kLaneWidth - lane)
            + kStationLength * abs(goal_.pose.position.x / kStationLength - station);
}

//check if point is on segment
bool Planner::onSegment(Point& start, Point& end, Point& pt) {
    return ((start.x - pt.x) * (end.x - pt.x) <= 0 && (start.y - pt.y) * (end.y - pt.y) <= 0);
}

//determine the side of point to the segment
int Planner::order(Point& start, Point& end, Point& pt) {
    double orientation = (end.y - start.y) * (pt.x - end.x) - (end.x - start.x) * (pt.y - end.y);
        if (!orientation) return 0;
        return orientation > 0 ? 1 : -1;
}

// check if two segments intersect
bool Planner::intersection(Point& seg1_start, Point& seg1_end, Point& seg2_start, Point& seg2_end) {
    int order1 = order(seg1_start, seg1_end, seg2_start);
    int order2 = order(seg1_start, seg1_end, seg2_end);
    int order3 = order(seg2_start, seg2_end, seg1_start);
    int order4 = order(seg2_start, seg2_end, seg1_end);
    if (order1 != order2 && order3 != order4) return true;
    if (!order1 && onSegment(seg1_start, seg1_end, seg2_start)) return true;
    if (!order2 && onSegment(seg1_start, seg1_end, seg2_end)) return true;
    if (!order3 && onSegment(seg2_start, seg2_end, seg1_start)) return true;
    if (!order4 && onSegment(seg2_start, seg2_end, seg1_end)) return true;
    return false;
}

// check if the ego vehicle collides with the obstacle. Safety margin is applied to both ego and obstacle.
bool Planner::checkCollision(double ego_x, double ego_y, double theta, double obs_x, double obs_y, an_messages::obstacle& obs) {
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    double half_length = kEgoHalfLength + kMargin / 2;
    double half_width = kEgoHalfWidth + kMargin / 2;
    std::vector<Point> ego_vertex{
            { ego_x + cos * half_length - sin * half_width, ego_y + sin * half_length + cos * half_width },
            { ego_x - cos * half_length - sin * half_width, ego_y - sin * half_length + cos * half_width },
            { ego_x - cos * half_length + sin * half_width, ego_y - sin * half_length - cos * half_width },
            { ego_x + cos * half_length + sin * half_width, ego_y + sin * half_length - cos * half_width } };
    std::vector<Point> obs_vertex{
            { obs_x + obs.length / 2 + kMargin / 2, obs_y + obs.width / 2 + kMargin / 2},
            { obs_x - obs.length / 2 - kMargin / 2, obs_y + obs.width / 2 + kMargin / 2},
            { obs_x - obs.length / 2 - kMargin / 2, obs_y - obs.width / 2 - kMargin / 2},
            { obs_x + obs.length / 2 + kMargin / 2, obs_y - obs.width / 2 - kMargin / 2}};
    // each time checks if one segment of ego bounding box intesects with another segment of obstacle bounding box
    for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                    if (intersection(ego_vertex[i], ego_vertex[(i + 1) % 4], obs_vertex[j], obs_vertex[(j + 1) % 4])) {
                            return true;
                    }
            }
    }
    return false;
}

// generate successors of current node using different motion primitives
std::vector<Node> Planner::generateSuccessors(const Node& node) {
    std::vector<Node> neighbors;
    for (int i = 0; i < motions_.size(); ++i) {
            int lane = node.lane + static_cast<int>((motions_[i].end[1] - motions_[i].start[1]) / kLaneWidth);
            int station = node.station + static_cast<int>((motions_[i].end[0] - motions_[i].start[0]) / kStationLength);
            if (lane >= 0 && lane < map_.size() && station < map_[0].size() && !map_[lane][station]) {
            //check if there is collision on trajectory
                bool is_collision = false;
                for (auto& obs:obstacles_.obs) {
                    if (is_collision) break;
                    int num_of_obs_traj = obs.path[0].traj.size();
                    for (auto& traj:motions_[i].trajectory) {
                        double ego_x = kStationLength * node.station + traj.x;
                        double ego_y = kLaneWidth * node.lane + traj.y;
                        double time = node.time + traj.time;
                        int time_step = static_cast<int>(time / kTimeStep);
                        double ratio = time / kTimeStep - time_step;
                        double obs_x, obs_y;
                        if (time_step >= num_of_obs_traj - 1) {
                            obs_x = obs.path[0].traj[num_of_obs_traj - 1].position.x;
                            obs_y = obs.path[0].traj[num_of_obs_traj - 1].position.y;
                        } else {
                            // interpolate obstacle position
                            obs_x = (1 - ratio) * obs.path[0].traj[time_step].position.x + ratio * obs.path[0].traj[time_step + 1].position.x;
                            obs_y = (1 - ratio) * obs.path[0].traj[time_step].position.y + ratio * obs.path[0].traj[time_step + 1].position.y;
                        }
                        // square of distance between centers of ego vehicle and obstacle
                        double r_square = (ego_x - obs_x) * (ego_x - obs_x) + (ego_y - obs_y) * (ego_y - obs_y);
                        if (r_square > (obs.length / 2 + kEgoHalfLength + kMargin) * (obs.length / 2 + kEgoHalfLength + kMargin)
                                + (obs.width / 2 + kEgoHalfWidth + kMargin) * (obs.width / 2 + kEgoHalfWidth + kMargin)) {
                            continue;
                        } else if (r_square < (obs.width / 2 + kEgoHalfWidth + kMargin) * (obs.width / 2 + kEgoHalfWidth + kMargin)) {
                            is_collision = true;
                            break;
                        } else if (checkCollision(ego_x, ego_y, traj.theta, obs_x, obs_y, obs)) {
                            is_collision = true;
                            break;
                        }
                    }
                }
                if (!is_collision) {
                    neighbors.emplace_back(Node{lane, station, node.time + motions_[i].end[3] - motions_[i].start[3],
                                                node.g_value + motions_[i].cost + heuristic(lane, station), node.g_value + motions_[i].cost, i});
                }
            }
    }
    return neighbors;
}


//Implements A* search
void Planner::plan() {
    //closed stores whether one node has been expanded and by which motion is it expanded
    std::vector<std::vector<std::vector<int>>> closed(map_.size(), std::vector<std::vector<int>>(map_[0].size(), std::vector<int>(2000, -1)));
    auto cmp = [](Node& left, Node& right) {return left.f_value == right.f_value ? left.time > right.time : left.f_value > right.f_value; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
    int start_lane = static_cast<int>(start_.pose.position.y / kLaneWidth);
    int start_station = static_cast<int>(start_.pose.position.x / kStationLength);
    int goal_lane = static_cast<int>(goal_.pose.position.y / kLaneWidth);
    int goal_station = static_cast<int>(goal_.pose.position.x / kStationLength);
    double goal_time = 0;
    open.push(Node{start_lane, start_station, 0, 0 + heuristic(start_lane, start_station), 0, 0});
    while (!open.empty()) {
        auto node = open.top();
        open.pop();
        int time_step = static_cast<int>(node.time / kTimeStep);
        //exits search when goal node is expanded
         if (node.lane == goal_lane && node.station == goal_station) {
             closed[node.lane][node.station][time_step] = node.motion_id;
             goal_time = node.time;
             break;
         }
         if (closed[node.lane][node.station][time_step] == -1) {
             closed[node.lane][node.station][time_step] = node.motion_id;
             auto neighbors = generateSuccessors(node);
             for (auto& neighbor : neighbors) {
                 int neighbor_time_step = static_cast<int>(neighbor.time / kTimeStep);
                 if (closed[neighbor.lane][neighbor.station][neighbor_time_step] == -1) {
                     open.push(neighbor);
                 }
            }
         }
    }

    //recursively find the path from goal position
    std::vector<std::vector<double>> path;
    int lane = goal_lane, station = goal_station;
    double time_elapsed = goal_time;
    while (lane != start_lane || station != start_station) {
        int time_step = static_cast<int>(time_elapsed / kTimeStep);
        int motion_id = closed[lane][station][time_step] == -1 ? closed[lane][station][time_step + 1] : closed[lane][station][time_step];
        auto& motion = motions_[motion_id];
        for (int i = motion.num_entries - 1; i >= 1; --i) {
            double ego_x = kStationLength * station + motion.trajectory[i].x - motion.end[0];
            double ego_y = kLaneWidth * lane + motion.trajectory[i].y - motion.end[1];
            double theta = motion.trajectory[i].theta - motion.end[2];
            double time = time_elapsed + motion.trajectory[i].time - motion.end[3];
            path.emplace_back(std::vector<double> {ego_x, ego_y, theta, time});
        }
        lane += static_cast<int>((motion.start[1] - motion.end[1]) / kLaneWidth);
        station += static_cast<int>((motion.start[0] - motion.end[0]) / kStationLength);
        time_elapsed += motion.start[3] - motion.end[3];
    }
    path.emplace_back(std::vector<double> {start_.pose.position.x, start_.pose.position.y, 0, time_elapsed});
    std::reverse(path.begin(), path.end());
    std::vector<an_messages::traj_pt> traj(path.size());
    for (int i = 0; i < path.size(); ++i) {
        traj[i].position.x = path[i][0];
        traj[i].position.y = path[i][1];
        traj[i].position.theta = path[i][2];
        traj[i].header.stamp = obstacles_.obs[0].path[0].traj[0].header.stamp + ros::Duration(path[i][3]);
    }
    //publish trajectory
    an_messages::trajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.traj = traj;
    pub_trajectory_.publish(trajectory);
}

void Planner::loop() {
    ros::Rate loop(loop_rate_);
    ros::Time start_time, end_time;
    while (ros::ok()) {
        if (new_plan_ && pose_received_ && goal_received_ && obstacle_received_ && map_received_) {
            ROS_WARN("[planner] Started Planning");
            start_time = ros::Time::now();
            new_plan_ = false;
            plan();
            end_time = ros::Time::now();
            double planning_time = 1e3 * (end_time - start_time).toSec();
            ROS_INFO_STREAM("[planner] Planning time (ms): " << planning_time);
            ROS_WARN("[planner] Finished planning");
        }
    ros::spinOnce();
    loop.sleep();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    //output debug info to screen
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG("[planner] Starting");
    Planner p1;
    if (p1.init()) {
        ROS_DEBUG("[planner] Entering loop");
        p1.loop();
    }
    ROS_DEBUG("[planner] Ending");
    return 0;
}
