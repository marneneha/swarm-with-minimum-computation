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
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_lib/ParamLoader.h>
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

  	void callbackOdomGt(const nav_msgs::OdometryConstPtr& msg);
	void callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
	void callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg);

	void collisionavoidance(std::string uav_name);
  	void activate(void);
	void neha_goto(float x,float y, std::string uav_name);
	void callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr& msg, const std::string& topic);
	void callbackuwbranging(const gtec_msgs::Ranging& msg, const std::string& topic);
	void callbacksonar(const /*msg type sonar*/& msg, const std::string& topic);
	void callbackimudata(const sensor_msgs::Imu& msg, const std::string& topic);
	double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz);
	int neighbourtimer(void);
	void callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);

	std::vector<ros::Subscriber>                            other_uav_coordinates;
  	std::vector<ros::Subscriber>                            other_uav_diag_subscribers;
  	std::vector<ros::Subscriber>                            other_uav_sonar;
  	std::vector<ros::Subscriber>                            other_uav_uwb_range;
  	std::vector<ros::Subscriber>                            other_uav_imu;
  	std::vector<std::string>                                other_drone_names_;
  	std::map<std::string, mrs_msgs::RtkGps> 		other_drones_location;
  	std::map<std::string, gtec_msgs::Ranging> 		other_drones_range;
  	std::map<std::string, /*msg type sonar*/> 		other_drones_sonar;
  	std::map<std::string, sensor_msgs::Imu> 		other_drones_imu;
	std::map<std::string, int>				other_drones_neighbour;
	std::map<std::string, int>				other_drones_preneighbour;
  	std::map<std::string, mrs_msgs::MpcTrackerDiagnostics>  other_drones_diagnostics;
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
	std::vector<mrs_msgs::ReferenceStamped>			new_waypoints;
	bool path_set=false;
  };
}




//init function
namespace neha_pkg1
{		ros::NodeHandle nh("~");
	void neha1::onInit()
	{
	//load parameter rom launch file
	mrs_lib::ParamLoader param_loader(nh, "neha1");
	//param_loader.load_param("uav_name", _uav_name_);
	param_loader.load_param("simulation", _simulation_);
	param_loader.load_param("frame_id", _frame_id_);
	//param_loader.load_param("network/robot_names", other_drone_names_);
	param_loader.load_param("tracker_diagnostics_in",tracker_diagnostics_in);
	other_drone_names_ = {"uav1", "uav2", "uav3", "uav4", "uav5", "uav6","uav7", "uav8", "uav9"};
	//subscrbing and publishing to respective topic 
	for (unsigned long i = 0; i < other_drone_names_.size(); i++) {
	//subscribe gps topic
	std::string prediction_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"rtk_gps"+std::string("/")+"global";
	other_uav_coordinates.push_back(nh.subscribe <mrs_msgs::RtkGps> (prediction_topic_name, 10, boost::bind(&neha1::callbackOtheruavcoordinates, this, _1, prediction_topic_name)));
 	ROS_INFO("[neha1]: subscribing to %s", prediction_topic_name.c_str());
	//subscribe diagnostics topic
	std::string diag_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") + "control_manager"+std::string("/")+"mpc_tracker"+std::string("/")+"diagnostics";    
	other_uav_diag_subscribers.push_back(nh.subscribe(diag_topic_name, 1, &neha1::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay()));
	ROS_INFO("[neha1]: subscribing to %s", diag_topic_name.c_str());				
	//advertise reference topic
	std::string neha_uav = "/"+other_drone_names_[i]+"/control_manager/reference";
	pub_reference_.push_back(nh.advertise<mrs_msgs::ReferenceStamped>(neha_uav,1));
	ROS_INFO("[neha1]:publishing to %s",neha_uav.c_str());
	//subscribe sonar topic
	std::string sonar_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+/*name of topic*/;
	other_uav_sonar.push_back(nh.subscribe </*msg type*/> (sonar_topic_name, 10, boost::bind(&neha1::callbacksonar, this, _1, sonar_topic_name)));
 	ROS_INFO("[neha1]: subscribing to %s", sonar_topic_name.c_str());
	//subscribe uwb topic  
	std::string uwb_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"gtec"+std::string("/")+"toa"+std::string("/")+"ranging";
	other_uav_uwb_range.push_back(nh.subscribe <gtec_msgs::Ranging> (uwb_topic_name, 10, boost::bind(&neha1::callbackuwbranging, this, _1, uwb_topic_name)));
 	ROS_INFO("[neha1]: subscribing to %s", uwb_topic_name.c_str());
	//suscribe imu topic  
	std::string imu_topic_name=std::string("/")+other_drone_names_[i]+std::string("/")+"mavros"+std::string("/")+"imu"+std::string("/")+"data_raw"/*or data*/;
	other_uav_imu.push_back(nh.subscribe <gtec_msgs::Ranging> (imu_topic_name, 10, boost::bind(&neha1::callbackimudata, this, _1, imu_topic_name)));
 	ROS_INFO("[neha1]: subscribing to %s", imu_topic_name.c_str());
	}
//------------subsriber---------------
	//ros::Subscriber sub_odom_gt_=nh.subscribe("odom_gt_in",1,&neha1::callbackOdomGt,this,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber sub_odom_uav_=nh.subscribe("odom_uav_in", 1, &neha1::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());

        
//---------------------timer------------------

timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(20), &neha1::callbackTimerPublishDistToWaypoint, this);
//------------------------publisher--------------

	
ROS_INFO_ONCE("m here in init");
		activate();	

		ros::spin();
	}



//activate fun this is fun to form a traingle of uav
void neha1::activate(void)
{
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
	new_waypoint.reference.yaw        = yaw;
	new_waypoints.push_back(new_waypoint);
	}

	  ROS_INFO("[neha1]: Flying to waypoint : x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f uav name: %s",new_waypoints[k].reference.position.x, new_waypoints[k].reference.position.y, new_waypoints[k].reference.position.z, new_waypoints[k].reference.yaw, uav_name.c_str() );
	k++;
	//ROS_INFO("m here in activate1 and value are n:%d,i:%d,j:%d ",n ,i, j);
    }
  }+u
	path_set=true;
	ROS_INFO("path has been set");
}
int l=0;
void neha1::callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te)
{
	
	if(path_set&&l<=2){
	pub_reference_[l].publish(new_waypoints[l]);
	l++;
	}
	else
	{
	l=0;
	}
}


//goto function






//localization algo 



//clear
//or mrs_msgs::RtkGps::ConstPtr&

void neha1::callbackOtheruavcoordinates(const mrs_msgs::RtkGpsConstPtr& msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  //std::string uav_name="uav"+uav_name-1;
  std::string uav_name="uav";
  other_drones_location[uav_name]=*msg;
}


//callback function for uwb sensor
void neha1::callbackuwbranging(const gtec_msgs::Ranging& msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  std::string uav_name="uav"+uav_no;
  //std::string uav_name="uav";
  tag[anchor] = msg->ranging_msg.anchorId;
  other_drones_range[tag] = msg->ranging_msg.tagId;
  other_drones_range[dist] = msg->ranging_msg.range;
  uwblocate();
}
void uwblocate(){
A = [(anchor2.pose.x-anchor1.pose.x) (anchor2.pose.y-anchor1.pose.y); (anchor3.pose.x-anchor2.pose.x) (anchor3.pose.y-anchor2.pose.y)];
B = [((anchor1.dist**2)-((tag.pose.z-anchor1.pose.z)**2)-(anchor1.pose.x**2)-(anchor1.pose.y**2))-((anchor2.dist**2)-((tag.pose.z-anchor2.pose.z)**2)-(anchor2.pose.x**2)-(anchor2.pose.y**2));
((anchor2.dist**2)-((tag.pose.z-anchor2.pose.z)**2)-(anchor2.pose.x**2)-(anchor2.pose.y**2))-((anchor3.dist**2)-((tag.pose.z-anchor3.pose.z)**2)-(anchor3.pose.x**2)-(anchor3.pose.y**2))];
X = inv(A)B;

}

//callback function for sonar 
void neha1::callbacksonar(const mrs_msgs::RtkGpsConstPtr& msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  std::string uav_name="uav"+uav_no;
  other_drones_sonar[uav_name]=*msg;
}
t=0;

//callback function for imu sensor
void neha1::callbackimudata(const sensor_msgs::Imu& msg, const std::string& topic){
  int uav_no = *(topic.c_str()+3); 
  std::string uav_name="uav"+uav_no;
  //std::string uav_name="uav";
  int t_0 = 0;
  int t = msg->header.stamp.nsecs;
  float dt = (t-t_0)/1000000000;
  t_0 = t;
  if(dt<1)
  x = x_0 + 0.5*(t**2)msg->linear_acceleration.x;
  y = y_0 + 0.5*(t**2)msg->linear_acceleration.y;
  z = z_0 + 0.5*(t**2)msg->linear_acceleration.z;
  cov = msg->linear_acceleration_covariance;
  x_0 = x;
  y_0 = y;
  z_0 = z;
  other_drones_imu[uav_name]=[x;y;z;cov];
}

//more time lag
//check no. of neighbour

//less time lag
//coliision avoidance
//check if timer has priority over while loop
//check where to use transfer of frame
//unnecesarry function


/*void neha1::callbackOdomGt(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomGt");
  odom_gt_ = *msg;		
}


void neha1::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackOdomUav");
  odom_uav_ = *msg;		
  }
*/

void neha1::callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg){
  ROS_INFO_ONCE("m here in callbackTrackerDiag");	
  other_drones_diagnostics[msg->uav_name] = *msg;  
  if (!msg->tracking_trajectory){
    ROS_INFO("[neha1]: %s Waypoint reached." ,other_drones_diagnostics[msg->uav_name]);
  }
}

double dist3d(const double ax, const double ay, const double az, const double bx, const double by, const double bz) {

  return sqrt(pow(ax - bx, 2) + pow(ay - by, 2) + pow(az - bz, 2));
}
}
PLUGINLIB_EXPORT_CLASS(neha_pkg1::neha1, nodelet::Nodelet);
