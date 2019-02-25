#include "ros/ros.h"
#include "an_messages/obstacle.h"
#include "an_messages/obstacles.h"
#include "an_messages/lane.h"
#include "an_messages/lanes.h"
#include "an_messages/traj_pt.h"
#include "an_messages/trajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

constexpr double kLaneWidth = 3.7;
constexpr double kStationLength = 10.0;
constexpr double kEgoHalfLength = 0.5 * 4.8;
constexpr double kEgoHalfWidth = 0.5 * 1.8;
constexpr double kMargin = 1.8;

struct Point {
    double x, y;
};

struct Node {
    int lane;
    int station;
    double f_value;
    double g_value;
    int motion_id; // the motion id by which the node is generated
};

struct PoseMprim {
    double x, y, theta;
    double time;
    double velocity;
};

struct Mprim {
    double start[4]; //start state
    double end[4];   //end state
    double cost;
    double length;
    int num_entries;
    std::vector<PoseMprim> trajectory;
};

class Planner {
private:
    std::vector<Mprim> motions_;
    bool new_plan_;
    bool pose_received_;
    bool goal_received_;
    bool obstacle_received_;
    bool map_received_;
    double loop_rate_;
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    an_messages::obstacles obstacles_;
    std::vector<std::vector<bool>> map_;

public:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_obstacles_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_trajectory_;

    void getPoseCallback (const geometry_msgs::PoseStamped::ConstPtr& msg);
    void getGoalCallback (const geometry_msgs::PoseStamped::ConstPtr& msg);
    void getObstaclesCallback (const an_messages::obstacles::ConstPtr& msg);
    void getMapCallback (const an_messages::lanes::ConstPtr& msg);
    void generateMap();
    bool loadMprim();
    bool init();
    double heuristic(int lane, int station, double epsilon = 1.0);
    bool onSegment(Point& start, Point& end, Point& pt);
    int order(Point& start, Point& end, Point& pt);
    bool intersection(Point& seg1_start, Point& seg1_end, Point& seg2_start, Point& seg2_end);
    bool checkCollision(double ego_x, double ego_y, double theta, an_messages::obstacle& obs);
    std::vector<Node> generateSuccessors(const Node& node);
    void plan();
    void loop();
};
