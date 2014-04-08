/*
* =====================================================================================
*
 *       Filename:  tangent_bug_with_waypoints.cpp
 *
 *    Description:  BUG2 algorithm implementation for ROS and VREP
 *
 *        Version:  1.0
 *        Created:  19/11/2013 15:59:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Roberto Marino (rm), formica@member.fsf.org
 *   Organization:  University of Genoa
 *
 * =====================================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <iterator>
#include <list>
#include <queue>
#include <sstream>
#include <string>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <ctime>
#include <cstdlib>

#define DTHRES 1	// Discontinuities Threshold (sort of resolution)
#define MIN_SAFETY_DISTANCE 0.5
#define NUM_OF_SCAN 360
#define MOVE_STEP 0.07
#define DEG_OFFSET_HOKUYO_WORLD 90
#define MAX_DISTANCE 100.0
#define MAX_RANGE_DETECTED 0.8 //previously 5.578
#define MIN_DISTANCE_TO_GOAL 0.3
#define TIME_INTERVAL 1000000
#define BLIND_MAX 20


#define DEBUG
//#define NO_MOTION



using namespace std;

typedef int vertex_t;
typedef double weight_t;
 
const weight_t max_weight = std::numeric_limits<double>::infinity();
 
struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

struct point{
	float x;
	float y;
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;

class IntervalOfContinuity
{
	public:
		int enddir1;
		int enddir2;
		geometry_msgs::Point endpoint1;
		geometry_msgs::Point endpoint2;
		
		float dist1;
		float dist2;
		float heurdist1;
		float heurdist2;
		int heurdist_min;
		float heurdist_min_value;
};


class Bug2Vrep
{
	public:
		Bug2Vrep();

		ros::Publisher Control_pub;

		ros::Publisher Motor1_pub;
		ros::Publisher Motor2_pub;
		ros::Publisher Motor3_pub;
		ros::Publisher Motor4_pub;

		ros::Publisher QuadTargetPosition_pub;

		ros::Publisher TangentLines_pub;
	

		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> cloudAbsolute;
		geometry_msgs::Point QuadPos;	// GET Variable
		geometry_msgs::Point GoalPos;
		geometry_msgs::Point GoalPosTemp;
			
		float ComputeDistance(geometry_msgs::Point x,geometry_msgs::Point y);
		float Deg2Rad(int AngDeg);
		int MinimumDistanceEndPointDir(void);
		void ComputeTangentDirections(void);
		void ComputeObstacleToGoal(void);	
		void ComputeAng2Goal(void);
		void exit_data(void);
			
		float AngRob2GoalRAD;	//Angle of the robot with rispect to the World frame (RAD) expressed in the world frame
		int AngRob2GoalDEG;	//Angle of the robot with rispect to the World frame (DEG) expressed in the world frame
		int AngRob2GoalDEG_prev;	

		float front_distance;	
		float dreach;
		float dfollowed;
		float dcurr;		
		float dprev;
		
		bool mot_2_goal;
		bool safety_distance_warning;
		bool goal_reached, circle_completed;		
		bool freePath,goTang,occObs,prevdir;
		bool hit_obstacle,new_leave_point;
		bool target_signal;
		bool goUP;
		bool blind;
		float distances[NUM_OF_SCAN];
		float distancesAbsolute[NUM_OF_SCAN];
		
		float QuadYaw;		// Yaw angle WRT the world frame in DEG

		int m_line_direction;
		float m_line_directionRAD;

		int blind_count;

		int last_hit_point_direction;
		int last_leave_point_direction;
		float last_hit_point_distance;

		int hit_point_counter;
		int leave_point_counter;
		int up_counter;
		float path_lenght;


		queue<IntervalOfContinuity> IOCs;

		int TangentDirections[100];	// Contain the directions angle of the connected segment (read couples)
		IntervalOfContinuity OccludingObstacle;		
		
		int NumOfObstacles;	
		int min_laser_distance_dir;		// Minimum distance percieved from the laser scanner
		int min_heuristic_distance_dir;	// Direction toward the discontinuity point with the minimum heuristic distance
		float min_laser_distance;
		// Graph data structures
		point nodes[10];
		adjacency_list_t adjacency_list;
		
	private:
		// Callbacks
                void RangeFinderPC2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void RangeFinderPC2AbsoluteCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
                void QuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void TargetSignalCallback(const std_msgs::Int32::ConstPtr& msg);
		adjacency_list_t CreateGraph(void);

		ros::NodeHandle nh_;
                
		ros::Subscriber RangeFinderPC2Absolute_sub;	// RangeFinder msgs from the robot
		ros::Subscriber RangeFinderPC2_sub;
                ros::Subscriber GoalPose_sub;		// Pose of the goal, useful to compute the distance
		ros::Subscriber QuadPose_sub;		// Pose of the robot from ground thruth or odometry 
		ros::Subscriber TargetSignal_sub;	
				
		sensor_msgs::PointCloud2 RangeData;	// GET Variable
		geometry_msgs::PoseStamped GoalPose;	// GET Variable
		geometry_msgs::Point TargetPos; 	// SET Variable
};


Bug2Vrep::Bug2Vrep()
{
       	RangeFinderPC2_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/vrep/RangeFinderDataPC2",10,&Bug2Vrep::RangeFinderPC2Callback,this); 
	RangeFinderPC2Absolute_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/vrep/RangeFinderDataPC2Absolute",10,&Bug2Vrep::RangeFinderPC2AbsoluteCallback,this); 
	GoalPose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vrep/GoalPose",10,&Bug2Vrep::GoalPoseCallback,this);
       	QuadPose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vrep/QuadPose",10,&Bug2Vrep::QuadPoseCallback,this);
	Control_pub = nh_.advertise<std_msgs::String>("/vrep/QuadMotorControl",1);	

	Control_pub = nh_.advertise<std_msgs::String>("/vrep/QuadMotorControl",1);	
		
	TargetSignal_sub = nh_.subscribe<std_msgs::Int32>("/vrep/TargetSignal",10,&Bug2Vrep::TargetSignalCallback,this);


	QuadTargetPosition_pub = nh_.advertise<geometry_msgs::Point>("/vrep/QuadrotorWaypointControl",1);
	
	Motor1_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor1",1);
	Motor2_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor2",1);
	Motor3_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor3",1);
	Motor4_pub = nh_.advertise<std_msgs::Float32>("/vrep/Motor4",1);

	TangentLines_pub = nh_.advertise<std_msgs::String>("/vrep/TangentLines",1);
	dcurr = MAX_DISTANCE;
	dprev = MAX_DISTANCE;
	circle_completed = 0;
	
	adjacency_list = CreateGraph();
}




void Bug2Vrep::TargetSignalCallback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("TargetSignal=%d", msg->data);
	target_signal = msg->data;
}

void Bug2Vrep::GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
        
	GoalPosTemp.x = msg->pose.position.x;
	GoalPosTemp.y = msg->pose.position.y;
	GoalPosTemp.z = msg->pose.position.z;
	
	if(!goUP)
	{	
		GoalPos.x = msg->pose.position.x;
		GoalPos.y = msg->pose.position.y;
		GoalPos.z = msg->pose.position.z;
	}	
	
}

void Bug2Vrep::QuadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	QuadPos.x = msg->pose.position.x;
	QuadPos.y = msg->pose.position.y;
	QuadPos.z = msg->pose.position.z; 
	

	QuadYaw = tf::getYaw(msg->pose.orientation);
	QuadYaw = QuadYaw*(180/3.14);	// WRT the WORLD FRAME in DEG
}

void Bug2Vrep::RangeFinderPC2AbsoluteCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg (*msg, cloudAbsolute);
	geometry_msgs::Point tmp;
		
	for(int i=0;i<cloudAbsolute.points.size(); ++i)
	{
		distancesAbsolute[i] = sqrt((GoalPos.x - cloudAbsolute.points[i].x)*(GoalPos.x - cloudAbsolute.points[i].x)+(GoalPos.y - cloudAbsolute.points[i].y)*(GoalPos.y - cloudAbsolute.points[i].y)); 
	}
	
}

#define NUM_OF_RND 20
#define OLD_VAR 12
#define VARIANCE 0.6


void Bug2Vrep::RangeFinderPC2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg (*msg, cloud);
	geometry_msgs::Point front_pnt, me;
	
	float noise,u;
	
	time_t seconds;
        time(&seconds);
        srand((unsigned int) seconds);

	noise=0;

	min_laser_distance = MIN_SAFETY_DISTANCE + 1;
	min_laser_distance_dir = 180;
	for(int i=0;i<cloud.points.size(); ++i)
	{
		u = 0;
		noise = 0;
		distances[i] = sqrt((cloud.points[i].x*cloud.points[i].x)+(cloud.points[i].y*cloud.points[i].y)); 
		// Adding simulated AWGN noise
		
		for(int k=0; k<NUM_OF_RND; k++)
		{
			u = 0.001*(rand()%1000);
			noise = noise + u;	
		}
		
		noise = noise - NUM_OF_RND/2;	// Set mean to 0
		noise = 0.1*noise*sqrt(VARIANCE);		// Set variance
		ROS_WARN("noise val: %f, rnd: %f",noise, u); 
	//	ROS_WARN("Prenoise Distance: %f",distances[i]);
	//	if(distances[i] != 0.0)		
		//	distances[i] += noise;
	//	ROS_WARN("Postnoise Distance: %f",distances[i]);	 
	}
	ComputeAng2Goal();
}

void Bug2Vrep::ComputeAng2Goal(void)
{
	AngRob2GoalRAD = atan2(GoalPos.y - QuadPos.y, GoalPos.x - QuadPos.x);			
	AngRob2GoalDEG = (int) (AngRob2GoalRAD * (180/3.14159));
	int Direction_in_robot_frame = AngRob2GoalDEG-(int)QuadYaw + 225;
}

float Bug2Vrep::Deg2Rad(int AngDeg)
{
	return (AngDeg*3.14/180);
}

float Bug2Vrep::ComputeDistance(geometry_msgs::Point a, geometry_msgs::Point b)
{
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void DijkstraComputePaths(vertex_t source,const adjacency_list_t &adjacency_list,std::vector<weight_t> &min_distance,std::vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));
 
    while (!vertex_queue.empty()) 
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());
 
        // Visit each edge exiting u
	const std::vector<neighbor> &neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(std::make_pair(min_distance[v], v));
 
	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(std::make_pair(min_distance[v], v));
 
	    }
 
        }
    }
}
 
 
std::list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t> &previous)
{
    std::list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}
 

float ComputeDistanceBetweenNode(point a, point b)
{
	return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

adjacency_list_t Bug2Vrep::CreateGraph(void)
{
	nodes[0].x = 0.875;
	nodes[0].y = 0.475;
	nodes[1].x = -2.65;
	nodes[1].y = 0.475;
	nodes[2].x = 6.15;
	nodes[2].y = -3.15;
	nodes[3].x = 2.05;
	nodes[3].y = -3.075;
	nodes[4].x = 1.175;
	nodes[4].y = -7.175;
	nodes[5].x = -2.55;
	nodes[5].y = -7.225;
	nodes[6].x = -4.925;
	nodes[6].y = -2.175;
	nodes[7].x = -7.6;
	nodes[7].y = -2.25;

	adjacency_list_t adjacency_list(10);
	// 0 = a
    	adjacency_list[0].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[0],nodes[1])));
    	adjacency_list[0].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[0],nodes[2])));    
	adjacency_list[0].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[0],nodes[3])));
	adjacency_list[0].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[0],nodes[4])));
	adjacency_list[0].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[0],nodes[5])));
	adjacency_list[0].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[0],nodes[6])));
	adjacency_list[0].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[0],nodes[7])));
  	
	//1 = b
	adjacency_list[1].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[1],nodes[0])));
    	adjacency_list[1].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[1],nodes[2])));    
	adjacency_list[1].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[1],nodes[3])));
	adjacency_list[1].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[1],nodes[4])));
	adjacency_list[1].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[1],nodes[5])));
	adjacency_list[1].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[1],nodes[6])));
	adjacency_list[1].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[1],nodes[7])));
	
	//2 = c
	adjacency_list[2].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[2],nodes[0])));
    	adjacency_list[2].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[2],nodes[1])));    
	adjacency_list[2].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[2],nodes[3])));
	adjacency_list[2].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[2],nodes[4])));
	adjacency_list[2].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[2],nodes[5])));
	adjacency_list[2].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[2],nodes[6])));
	adjacency_list[2].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[2],nodes[7])));
	
   	//3 = d
	adjacency_list[3].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[3],nodes[0])));
    	adjacency_list[3].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[3],nodes[1])));    
	adjacency_list[3].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[3],nodes[2])));
	adjacency_list[3].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[3],nodes[4])));
	adjacency_list[3].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[3],nodes[5])));
	adjacency_list[3].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[3],nodes[6])));
	adjacency_list[3].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[3],nodes[7])));
	
    	//4 = e
	adjacency_list[4].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[4],nodes[0])));
    	adjacency_list[4].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[4],nodes[1])));    
	adjacency_list[4].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[4],nodes[2])));
	adjacency_list[4].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[4],nodes[3])));
	adjacency_list[4].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[4],nodes[5])));
	adjacency_list[4].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[4],nodes[6])));
	adjacency_list[4].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[4],nodes[7])));
	
  	//5 = f
	adjacency_list[5].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[5],nodes[0])));
    	adjacency_list[5].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[5],nodes[1])));    
	adjacency_list[5].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[5],nodes[2])));
	adjacency_list[5].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[5],nodes[3])));
	adjacency_list[5].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[5],nodes[4])));
	adjacency_list[5].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[5],nodes[6])));
	adjacency_list[5].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[5],nodes[7])));
	
  	//6 = g
	adjacency_list[6].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[6],nodes[0])));
    	adjacency_list[6].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[6],nodes[1])));    
	adjacency_list[6].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[6],nodes[2])));
	adjacency_list[6].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[6],nodes[3])));
	adjacency_list[6].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[6],nodes[4])));
	adjacency_list[6].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[6],nodes[5])));
	adjacency_list[6].push_back(neighbor(7, ComputeDistanceBetweenNode(nodes[6],nodes[7])));
	
	//7 = h
	adjacency_list[7].push_back(neighbor(0, ComputeDistanceBetweenNode(nodes[7],nodes[0])));
    	adjacency_list[7].push_back(neighbor(1, ComputeDistanceBetweenNode(nodes[7],nodes[1])));    
	adjacency_list[7].push_back(neighbor(2, ComputeDistanceBetweenNode(nodes[7],nodes[2])));
	adjacency_list[7].push_back(neighbor(3, ComputeDistanceBetweenNode(nodes[7],nodes[3])));
	adjacency_list[7].push_back(neighbor(4, ComputeDistanceBetweenNode(nodes[7],nodes[4])));
	adjacency_list[7].push_back(neighbor(5, ComputeDistanceBetweenNode(nodes[7],nodes[5])));
	adjacency_list[7].push_back(neighbor(6, ComputeDistanceBetweenNode(nodes[7],nodes[6])));
	
	return adjacency_list;
	
}

void Bug2Vrep::exit_data(void)
{
	ROS_WARN("----------------------------");
	ROS_WARN("PathLenght: %f", path_lenght);
	ROS_WARN("Hit Points: %d", hit_point_counter);
	ROS_WARN("Leave Points: %d", leave_point_counter);
	ROS_WARN("Up Motions: %d", up_counter);
	ROS_WARN("Residual Distance from Goal: %f", dcurr);
	ROS_WARN("----------------------------");
}

int main(int argc, char** argv)
{
        ros::init(argc,argv, "Bug2Vrep");
        Bug2Vrep bvrep;
  
	geometry_msgs::Point TargetPosition;
	geometry_msgs::Point Delta;
	geometry_msgs::Point HitPoint;
	geometry_msgs::Point LeavePoint;
	
	bvrep.mot_2_goal = 1; // The robot start in motion_to_goal behav
	bvrep.hit_obstacle = 0;
	bvrep.circle_completed = 0;
	bvrep.new_leave_point = 0;
	int t = 0;

	bvrep.GoalPos.x = bvrep.GoalPosTemp.x;
	bvrep.GoalPos.y = bvrep.GoalPosTemp.y;
	bvrep.goUP = 0;
	bvrep.blind = 0;
	bvrep.blind_count = BLIND_MAX;

	bvrep.hit_point_counter = 0;
	bvrep.leave_point_counter = 0;
	bvrep.up_counter = 0;
	bvrep.path_lenght = 0;

	
	while(ros::ok())
	{
			
		//-- Navigation algorithm
		if(bvrep.mot_2_goal)
		{
			ros::spinOnce();
			TargetPosition.x = bvrep.QuadPos.x;
			TargetPosition.y = bvrep.QuadPos.y;
			TargetPosition.z = bvrep.QuadPos.z;
			bvrep.hit_obstacle = 0;
	
			if(t==1)
			{
				// Compute the m_line direction from START to GOAL
				bvrep.m_line_direction = bvrep.AngRob2GoalDEG;
				bvrep.m_line_directionRAD = bvrep.AngRob2GoalRAD;
				ROS_WARN("M-LINE DIRECTION: %d", bvrep.m_line_direction);
				
			}

			if(t!=0)
			{
				for(int i=0; i<360;i++)
				{	
					if(!bvrep.blind)
					{
					//ROS_INFO("Distances[%d]=%f",i,bvrep.distances[i]);
						if((bvrep.distances[i]<MIN_SAFETY_DISTANCE) && (bvrep.distances[i] != 0.0))
						{
			//				ROS_WARN("Distance %d is not safe",i);
							bvrep.hit_obstacle=1;
						}
					}
				}	
				
			}	
			
			if(bvrep.blind)
			{
				bvrep.blind_count--;
				if(bvrep.blind_count == 0)
				{
					bvrep.blind = 0;
					ROS_WARN("Now I see AGAIN!");
				}
			}
			
			//int temp_ang = bvrep.AngRob2GoalDEG+(int)bvrep.QuadYaw-225;	// Navigatin on the m-line
			int temp_ang = bvrep.AngRob2GoalDEG;
#ifdef DEBUG				
			ROS_WARN("Motion 2 Goal! AngRob2GoalDEG=%d",bvrep.AngRob2GoalDEG);
#endif
			if(temp_ang > 360)
				temp_ang -= 360;
			else if(temp_ang < 360)
				temp_ang += 360;
			
			if(!bvrep.hit_obstacle)
			{
			//	TargetPosition.x+=MOVE_STEP*cos(bvrep.Deg2Rad(temp_ang));
			//	TargetPosition.y+=MOVE_STEP*sin(bvrep.Deg2Rad(temp_ang));
		//		TargetPosition.x+=MOVE_STEP*cos(bvrep.m_line_directionRAD);
		//		TargetPosition.y+=MOVE_STEP*sin(bvrep.m_line_directionRAD);
				TargetPosition.x+=MOVE_STEP*cos(bvrep.AngRob2GoalRAD);
				TargetPosition.y+=MOVE_STEP*sin(bvrep.AngRob2GoalRAD);
				bvrep.path_lenght += MOVE_STEP;

#ifndef NO_MOTION					     
				bvrep.QuadTargetPosition_pub.publish(TargetPosition);		// Send the control signal
#endif			
			}   
			
			
				
			usleep(TIME_INTERVAL);
			
			bvrep.dcurr = bvrep.ComputeDistance(bvrep.GoalPos,bvrep.QuadPos);
		
			if((bvrep.dcurr < MIN_DISTANCE_TO_GOAL) && (bvrep.dcurr != 0.0) && bvrep.target_signal)
			{ 
				ROS_WARN("Goal Reached: %f", bvrep.dcurr);
				bvrep.exit_data();	
				ros::shutdown();
				return 0;		
			}	
		
			if(bvrep.goUP && (bvrep.dcurr < MIN_DISTANCE_TO_GOAL) && (bvrep.dcurr != 0.0))
			{
				ROS_WARN("GO_UP");
				TargetPosition.z += 2;
				bvrep.GoalPos.x = bvrep.GoalPosTemp.x;
				bvrep.GoalPos.y = bvrep.GoalPosTemp.y;
				bvrep.goUP = 0;
				bvrep.path_lenght += 2 ;
				bvrep.up_counter++;
				bvrep.QuadTargetPosition_pub.publish(TargetPosition);
				usleep(10000000);
				bvrep.dcurr = bvrep.ComputeDistance(bvrep.GoalPos,bvrep.QuadPos);
			}
	
			if((bvrep.dcurr < MIN_DISTANCE_TO_GOAL) && (bvrep.dcurr != 0.0) && (bvrep.goUP == 0))
			{
				bvrep.nodes[8].x = bvrep.QuadPos.x;
				bvrep.nodes[8].y = bvrep.QuadPos.y;
				//8 = Actual Position - Edges from the actual position to the holes	
				bvrep.adjacency_list[8].push_back(neighbor(0, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[0])));
				bvrep.adjacency_list[8].push_back(neighbor(1, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[1])));    
				bvrep.adjacency_list[8].push_back(neighbor(2, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[2])));
				bvrep.adjacency_list[8].push_back(neighbor(3, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[3])));
				bvrep.adjacency_list[8].push_back(neighbor(4, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[4])));
				bvrep.adjacency_list[8].push_back(neighbor(5, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[5])));
				bvrep.adjacency_list[8].push_back(neighbor(6, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[6])));
				bvrep.adjacency_list[8].push_back(neighbor(7, ComputeDistanceBetweenNode(bvrep.nodes[8],bvrep.nodes[7])));
				// New Goal (through the waypoints)
				bvrep.nodes[9].x = bvrep.GoalPos.x;
				bvrep.nodes[9].y = bvrep.GoalPos.y;
				// Edges from the holes to the goal
				bvrep.adjacency_list[0].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[0],bvrep.nodes[9])));
				bvrep.adjacency_list[1].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[1],bvrep.nodes[9])));    
				bvrep.adjacency_list[2].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[2],bvrep.nodes[9])));
				bvrep.adjacency_list[3].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[3],bvrep.nodes[9])));
				bvrep.adjacency_list[4].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[4],bvrep.nodes[9])));
				bvrep.adjacency_list[5].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[5],bvrep.nodes[9])));
				bvrep.adjacency_list[6].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[6],bvrep.nodes[9])));
				bvrep.adjacency_list[7].push_back(neighbor(9, ComputeDistanceBetweenNode(bvrep.nodes[7],bvrep.nodes[9])));
				
				std::vector<weight_t> min_distance;
				std::vector<vertex_t> previous;
				DijkstraComputePaths(8, bvrep.adjacency_list, min_distance, previous);
				std::cout << "Distance(ActualPosition,NewGoal) = " << min_distance[9] << std::endl;
				std::list<vertex_t> path = DijkstraGetShortestPathTo(9, previous);
				std::cout << "Path : ";
				std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
				std::cout << std::endl;
				int* array=new int[path.size()]; // create a dynamic array
    				std::copy(path.begin(),path.end(),array); // copy the data
				ROS_INFO("Position of the waypoints (%f,%f)", bvrep.nodes[array[1]].x, bvrep.nodes[array[1]].y);
				bvrep.GoalPos.x = bvrep.nodes[array[1]].x;
				bvrep.GoalPos.y = bvrep.nodes[array[1]].y;
				bvrep.goUP = 1;
				delete [] array; // destroy the dynamic array

				
			}
						
			if(bvrep.hit_obstacle)
			{									// Obstacle encountered
				ROS_WARN("Hit obstacle!");
				bvrep.mot_2_goal = 0;
				bvrep.circle_completed = 0;
				HitPoint.x = bvrep.QuadPos.x;
				HitPoint.y = bvrep.QuadPos.y;
				bvrep.last_hit_point_direction = bvrep.AngRob2GoalDEG;
				bvrep.last_hit_point_distance = bvrep.dcurr;
				bvrep.m_line_direction = bvrep.AngRob2GoalDEG;
				bvrep.m_line_directionRAD = bvrep.AngRob2GoalRAD;
				ROS_WARN("M-LINE DIRECTION: %d", bvrep.m_line_direction);
				bvrep.hit_point_counter++;
			}
		}
	
		if(!bvrep.mot_2_goal)
		{
			ros::spinOnce();
			//bvrep.hit_obstacle = 0;
			TargetPosition.x = bvrep.QuadPos.x;
			TargetPosition.y = bvrep.QuadPos.y;
			TargetPosition.z = bvrep.QuadPos.z;
#ifdef DEBUG				
			ROS_WARN("Boundary Following. AngRob2GoalDEG=%d", bvrep.AngRob2GoalDEG);
#endif
			bvrep.min_laser_distance = MIN_SAFETY_DISTANCE;
			
			for(int i=0; i<359;i++)
			{	
				if((bvrep.distances[i]<bvrep.min_laser_distance) && (bvrep.distances[i] != 0.0))
				{	
					bvrep.min_laser_distance = bvrep.distances[i];
					bvrep.min_laser_distance_dir = i;
					bvrep.mot_2_goal=0;
				}
			}

			// ----- VERY IMPORTANT CODE
			//float alphax = atan2((MIN_SAFETY_DISTANCE-bvrep.min_laser_distance),MOVE_STEP);
			//alphax = alphax * (180/3.14); //Convert in deg
			//int temp_angle = bvrep.min_laser_distance_dir+(int)bvrep.QuadYaw-225-135-(int)alphax;
			// ----- END VERY IMPORTANT CODE
			if(bvrep.min_laser_distance>0.4)
			{	
				ROS_INFO("Trajectory Correction");
				int temp_angle = bvrep.min_laser_distance_dir+(int)bvrep.QuadYaw-225; //Trajectory correction
				if(temp_angle>360)
					temp_angle = temp_angle - 360;
				if(temp_angle<0)
					temp_angle = temp_angle + 360;
				TargetPosition.x+=0.05*cos(bvrep.Deg2Rad(temp_angle));
				TargetPosition.y+=0.05*sin(bvrep.Deg2Rad(temp_angle));	
			} 
			else
			{
				int temp_angle = bvrep.min_laser_distance_dir+(int)bvrep.QuadYaw-225-90; //Go tangent to the boundary

				if(temp_angle>360)
					temp_angle = temp_angle - 360;
				if(temp_angle<0)
					temp_angle = temp_angle + 360;
				TargetPosition.x+=0.05*cos(bvrep.Deg2Rad(temp_angle));
				TargetPosition.y+=0.05*sin(bvrep.Deg2Rad(temp_angle));
			}
			
			if(bvrep.min_laser_distance<0.3)
			{	
				ROS_INFO("Trajectory Correction");
				int temp_angle = bvrep.min_laser_distance_dir+(int)bvrep.QuadYaw-225-180; //Trajectory correction
				if(temp_angle>360)
					temp_angle = temp_angle - 360;
				if(temp_angle<0)
					temp_angle = temp_angle + 360;
				TargetPosition.x+=0.05*cos(bvrep.Deg2Rad(temp_angle));
				TargetPosition.y+=0.05*sin(bvrep.Deg2Rad(temp_angle));	
			 }
#ifndef NO_MOTION		
			bvrep.QuadTargetPosition_pub.publish(TargetPosition);
			bvrep.path_lenght += 0.05;
#endif				
			usleep(TIME_INTERVAL);		
				
			bvrep.dcurr = bvrep.ComputeDistance(bvrep.GoalPos,bvrep.QuadPos);
	
			if((bvrep.dcurr < MIN_DISTANCE_TO_GOAL) && (bvrep.dcurr != 0.0) && bvrep.target_signal)
			{ 
				ROS_WARN("Goal Reached: %f", bvrep.dcurr);	
				bvrep.exit_data();
				ros::shutdown();
				return 0;						
			}	
			if(bvrep.circle_completed)
			{
				ROS_INFO("Goal Reached or Circle Completed!");
				bvrep.exit_data();
				ros::shutdown();
				return 0;
			}
			if(( bvrep.AngRob2GoalDEG < bvrep.m_line_direction) && (bvrep.m_line_direction > bvrep.AngRob2GoalDEG_prev) && (bvrep.dcurr < bvrep.last_hit_point_distance)) // LeavePoint
			{
				ROS_WARN("Leave Point Dir: %d",bvrep.AngRob2GoalDEG);
				bvrep.mot_2_goal = 1;
				bvrep.circle_completed = 1;
				LeavePoint.x = bvrep.QuadPos.x;
				LeavePoint.y = bvrep.QuadPos.y;
				bvrep.blind = 1;
				bvrep.blind_count = BLIND_MAX;
				bvrep.m_line_direction = bvrep.AngRob2GoalDEG;
				bvrep.m_line_directionRAD = bvrep.AngRob2GoalRAD;
				ROS_WARN("M-LINE DIRECTION: %d", bvrep.m_line_direction);
				bvrep.leave_point_counter++;
			}
			
			bvrep.AngRob2GoalDEG_prev = bvrep.AngRob2GoalDEG;
		}
	
	t+=1;
	
	}
	bvrep.exit_data();
	printf("Bug+Dijkstra Algorithm ended!\n");
	ros::shutdown();
	return(0);
}


