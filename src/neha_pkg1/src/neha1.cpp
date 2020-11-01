//these are all header file declaration
//#define _GLIBCXX_USE_CXX11_ABI 0
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
//#include <mrs_msgs/TrackerPoint.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/RtkGps.h>
#include <sensor_msgs/LaserScan.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Trigger.h>
#include <math.h>
#include <string.h>
#include <map> 
#include <iterator> 
#include <std_msgs/String.h> 
namespace neha_pkg1
{

  class neha1 : public nodelet::Nodelet
  {
  public:
	virtual void onInit();

  	//void callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
	//void callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
	void callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr msg, const std::string& topic);
	void callbackLidar(const sensor_msgs::LaserScanConstPtr msg, const std::string& topic);
	void callbackGroundTruth(const nav_msgs::OdometryConstPtr msg,const std::string& topic);
	void collisionavoidance(std::string uav_name);
  	void activate(void);
	void neha_goto(float x,float y, std::string uav_name);
	void callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr msg, const std::string& topic);
	double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz);
	void neighbourtimer(const ros::TimerEvent& te);
	void callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);

	std::vector<ros::Subscriber>                            other_uav_coordinates;
  	std::vector<ros::Subscriber>                            sub_uav_diag;
	std::vector<ros::Subscriber>                            sub_uav_lidar;
	std::vector<ros::Subscriber>                            sub_gt;
  	std::vector<std::string>                                other_drone_names_;
  	std::map<std::string, mrs_msgs::RtkGps> 		other_drones_location;
	std::map<std::string, int>				other_drones_neighbour;
	std::map<std::string, int>				other_drones_preneighbour;
  	std::map<std::string, bool>  				other_drones_diagnostics;
	std::map<int, geometry_msgs::Point>			current_position;
	std::map<int, geometry_msgs::Quaternion>		current_orintation;
	std::string 						rtk_gps;
	std::string 						global;
	std::string						control_manager;
	std::string						mpc_tracker;
	std::string						diagnostics;
	std::string						tracker_diagnostics_in;
	//nav_msgs::Odometry					odom_gt_;
  	//nav_msgs::Odometry 					odom_uav_;
	std::string _frame_id_;
	bool _simulation_;
	std::vector<ros::Publisher>				pub_reference_;
	ros::Timer 						timer_publish_dist_to_waypoint_;
	ros::Timer 						timer_neighbour_;
	std::vector<mrs_msgs::ReferenceStamped>			new_waypoints;
	bool path_set=false;
	bool obstacle;
	bool coherence;
  };
}




//init function
namespace neha_pkg1
{		ros::NodeHandle nh("~");
	void neha1::onInit()
	{

	mrs_lib::ParamLoader param_loader(nh, "neha1");
	//param_loader.load_param("uav_name", _uav_name_);
	param_loader.loadParam("simulation", _simulation_);
	param_loader.loadParam("frame_id", _frame_id_);
	ROS_INFO("m here123");
	//param_loader.load_param("network/robot_names", other_drone_names_);
	ROS_INFO("m here456");
	param_loader.loadParam("tracker_diagnostics_in",tracker_diagnostics_in);
		other_drone_names_ = {"uav1", "uav2", "uav3"};
		//other_drone_names_ = {"uav1"};

	for (unsigned long i = 0; i < other_drone_names_.size(); i++) {
	std::string prediction_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"rtk_gps"+std::string("/")+"global";
	other_uav_coordinates.push_back(nh.subscribe <mrs_msgs::RtkGps> (prediction_topic_name, 10, boost::bind(&neha1::callbackOtheruavcoordinates, this, _1, prediction_topic_name)));
 	ROS_INFO("[neha1]: subscribing to %s", prediction_topic_name.c_str());

	std::string diag_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") + "control_manager"+std::string("/")+"diagnostics";    
	sub_uav_diag.push_back(nh.subscribe <mrs_msgs::ControlManagerDiagnostics> (diag_topic_name, 20, boost::bind(&neha1::callbackTrackerDiag, this, _1, diag_topic_name)));
	ROS_INFO("[neha1]: subscribing to %s", diag_topic_name.c_str());				
	
	std::string lidar_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") + "rplidar"+std::string("/")+"scan";
	//correct the msg type here
	sub_uav_lidar.push_back(nh.subscribe <sensor_msgs::LaserScan> (lidar_topic_name, 10, boost::bind(&neha1::callbackLidar, this, _1, lidar_topic_name)));
	ROS_INFO("[neha1]: subscribing to %s", lidar_topic_name.c_str());

	std::string gt_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") + "odometry" + std::string("/") + "odom_main";
	sub_gt.push_back(nh.subscribe <nav_msgs::Odometry> (gt_topic_name, 10, boost::bind(&neha1::callbackGroundTruth, this, _1, gt_topic_name)));
	ROS_INFO("[neha1]:publishing to %s",gt_topic_name.c_str());

	std::string neha_uav = "/"+other_drone_names_[i]+"/control_manager/reference";
	pub_reference_.push_back(nh.advertise<mrs_msgs::ReferenceStamped>(neha_uav,1));
	ROS_INFO("[neha1]:publishing to %s",neha_uav.c_str());

	}
//------------subsriber---------------
	//ros::Subscriber sub_odom_gt_=nh.subscribe("odom_gt_in",1,&neha1::callbackOdomGt,this,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber sub_odom_uav_=nh.subscribe("odom_uav_in", 1, &neha1::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());

        
//---------------------timer------------------

timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(10), &neha1::callbackTimerPublishDistToWaypoint, this);
timer_neighbour_ = nh.createTimer(ros::Rate(10), &neha1::neighbourtimer, this);
//------------------------publisher--------------

	
ROS_INFO_ONCE("m here in init");
		activate();	

		ros::spin();
	}


//see this thing works mostly it should as while ros::ok() works with subsrciber & timer if it does not add that if code inside void neha1::callbackTimerPublishDistToWaypoint() 
//activate fun this is fun to form a traingle of uav
int count1, count2;
void neha1::activate(void)
{
	while(ros::ok()){
	int x=0,y=0,i=0,j=0,N=3;
	float z=3,yaw=3.14;
	int n = (sqrt(8*N+1)-1)/2;
	int k=0;
	mrs_msgs::ReferenceStamped new_waypoint;
	  for (i=0;i<n;i++){
	    for (j=0;j<=i;j++){
		std::string uav_name=other_drone_names_[k];

		float x=10*(-1*0.2*i+0.8*j);
		float y=10*(-1*0.8*i);

		float z=5,yaw=0.23;

		{
		new_waypoint.header.frame_id = uav_name +"/"+ _frame_id_;
		ROS_INFO("hii neha its me %s",uav_name.c_str());	
		new_waypoint.header.stamp    = ros::Time::now();
		new_waypoint.reference.position.x = x;
		new_waypoint.reference.position.y = y;
		new_waypoint.reference.position.z = z;
		new_waypoint.reference.heading    = yaw;
		new_waypoints.push_back(new_waypoint);
		}

		  ROS_INFO("[neha1]: Flying to waypoint : x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f uav name: %s",new_waypoints[k].reference.position.x, new_waypoints[k].reference.position.y, new_waypoints[k].reference.position.z, new_waypoints[k].reference.heading, uav_name.c_str() );
		k++;
		//ROS_INFO("m here in activate1 and value are n:%d,i:%d,j:%d ",n ,i, j);
	    }
	  }
		path_set=true;
		ROS_INFO("path has been set");
	std::cout << __FILE__ << ":" << __LINE__  << " m in while for i="  <<count1<<std::endl; 
	
		ros::spin();
     }
  }
  

int l=0;

void neha1::callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te)
{
	std::cout << __FILE__ << ":" << __LINE__  << " reaching state of uav1 "  <<other_drones_diagnostics["uav1"]<<std::endl; 
	std::cout << __FILE__ << ":" << __LINE__  << " reaching state of uav2 "  <<other_drones_diagnostics["uav2"]<<std::endl; 
	std::cout << __FILE__ << ":" << __LINE__  << " reaching state of uav3 "  <<other_drones_diagnostics["uav3"]<<std::endl; 

		if(path_set&&l<=2){
		pub_reference_[l].publish(new_waypoints[l]);
		l++;
		}
		else
		{
		l=0;
		}
	if(other_drones_diagnostics["uav1"]&&other_drones_diagnostics["uav2"]&&other_drones_diagnostics["uav3"] && !obstacle && coherence)
	{
		new_waypoints[0].reference.position.y = new_waypoints[0].reference.position.y+15;
		new_waypoints[1].reference.position.y = new_waypoints[1].reference.position.y+15;
		new_waypoints[2].reference.position.y = new_waypoints[2].reference.position.y+15;
	}
	  std::cout << __FILE__ << ":" << __LINE__  << " m in callback timer for i="  <<count2<<std::endl; 
	count2++;
}


//goto function

//clear
//or mrs_msgs::RtkGps::ConstPtr&


void neha1::callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  //std::string uav_name="uav"+uav_name-1;
  std::string uav_name="uav";
  other_drones_location[uav_name]=*msg;
}


void neha1::neighbourtimer(const ros::TimerEvent& te){
coherence = true;
for(int i=0; i<2;i++){
	for(int j=i+1; j<=2;j++){
	float dist1 = dist3d(current_position[i].x,current_position[i].y,current_position[i].z,current_position[j].x,current_position[j].y,current_position[j].z);
	if (dist1>5 || dist1<2){
		//move reverse 
		coherence = false;
		new_waypoints[i].reference.position.x = current_position[i].x-cos(current_orintation[i].z);		  
		new_waypoints[i].reference.position.y = current_position[i].y-sin(current_orintation[i].z);		  
		new_waypoints[j].reference.position.x = current_position[j].x-cos(current_orintation[i].z);		  
		new_waypoints[j].reference.position.y = current_position[j].y-sin(current_orintation[i].z);		  

		}
	}
  }
}




void neha1::callbackLidar(const sensor_msgs::LaserScanConstPtr msg, const std::string& topic){
  ROS_INFO_ONCE("m here in callback lidar");
  int uav_no = *(topic.c_str()+4);
  uav_no = uav_no-49;
  obstacle = false;
	for(int i=0; i<=355; i++){
		if (msg->ranges[i]<5){ 
		obstacle = true;
		new_waypoints[uav_no].reference.position.x = current_position[uav_no].x+cos(90+i);		  
		new_waypoints[uav_no].reference.position.y = current_position[uav_no].y+sin(90+i);		  

		}
	}
	 
}

//unnecesarry function
void neha1::callbackGroundTruth(const nav_msgs::OdometryConstPtr msg,const std::string& topic){
int uav_no = *(topic.c_str()+4);
uav_no = uav_no-49;
current_position[uav_no] = msg->pose.pose.position;
current_orintation[uav_no] = msg->pose.pose.orientation;
}



void neha1::callbackTrackerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr msg, const std::string& topic){
  ROS_INFO_ONCE("m here in callbackTrackerDiag");
  int uav_no = *(topic.c_str()+4);
  uav_no = uav_no-48;
  std::string uav_name="uav"+std::to_string(uav_no);	
  other_drones_diagnostics[uav_name] = msg->tracker_status.have_goal;  
  if (!msg->tracker_status.have_goal){
  std::cout << __FILE__ << ":" << __LINE__ << uav_name << "with number" << uav_no <<"waypoint reached "  <<std::endl; 
    

  }
}


double neha1::dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz) {

  return sqrt(pow(ax - bx, 2) + pow(ay - by, 2) + pow(az - bz, 2));
}


/*void neha1::callbackOdomGt(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomGt");
  odom_gt_ = *msg;		
}


void neha1::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomUav");
  odom_uav_ = *msg;		
  }
*/

/*
//more time lag
//check no. of neighbour
int neha1::neighbourtimer(void){
  float R=1.0;
  int neighbour=0;
  std::map<std::string, mrs_msgs::RtkGps>::iterator u = other_drones_location.begin();
  std::map<std::string, mrs_msgs::RtkGps>::iterator v = other_drones_location.begin();
  while (u != other_drones_location.end()) {
    neighbour=0;
    while (v != other_drones_location.end()) {
      if(dist3d(u->second.pose.pose.position.x, u->second.pose.pose.position.y, u->second.pose.pose.position.z,v->second.pose.pose.position.x,v->second.pose.pose.position.y,v->second.pose.pose.position.z)<R){
        neighbour++;
      }
    v++;
    }
    other_drones_neighbour[u->first] = neighbour;
    u++;
  }
}
//less time lag
//coliision avoidance
void neha1::collisionavoidance(std::string uav_name){
//check if timer has priority over while loop
//check where to use transfer of frame
float dt = 0.1,x=0,y=0; int alpha =2;
std::map<std::string, int>::iterator u = other_drones_neighbour.begin();
std::map<std::string,mrs_msgs::RtkGps>::iterator v = other_drones_location.begin();
   while (u != other_drones_neighbour.end()) {
     if((u->second)<alpha){
	uav_name=u->first;
	if(u->second < other_drones_preneighbour[uav_name]){
	x=-v->second.twist.twist.linear.x*dt;
	y=-v->second.twist.twist.linear.y*dt;
	neha_goto(x,y,uav_name);
	}
    }
    else{
//this is resetting target
	neha_goto(x,y,uav_name);
    }
  other_drones_preneighbour[uav_name]=u->second;
  u++;
  }
}*/


}
PLUGINLIB_EXPORT_CLASS(neha_pkg1::neha1, nodelet::Nodelet);
