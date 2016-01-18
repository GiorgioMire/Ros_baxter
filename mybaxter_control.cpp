#include <math.h>
#include <ros/ros.h>
 #include <tf/tf.h>
 #include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
 #include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <baxter_core_msgs/JointCommand.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <sstream>
#include <iostream>
#include <Eigen/Core>
#include <armadillo>

using namespace arma;

using namespace std;

void attendi(){
 //cout<<std::endl<<std::endl<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"<<std::endl<<std::endl;
  char a;
  while(!(a=getchar())){};
  if(a=='a')
    ROS_ERROR("uscita");

};  
typedef baxter_core_msgs::JointCommand jcmd;


void mylookupTwist(  tf::TransformListener& l,const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
  const ros::Time& time, const ros::Duration& averaging_interval, 
  geometry_msgs::Twist& twist) 
{
 ros::Time latest_time, target_time;
 l.getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); 
 cout<<endl<<"latest_time"<<latest_time<<endl;


 if (ros::Time() == time)
   target_time = latest_time;
 else
  target_time = time;

ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);

  ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
  tf::StampedTransform start, end;


  l.waitForTransform (observation_frame, tracking_frame, start_time, ros::Duration(0.5));
  l.lookupTransform(observation_frame, tracking_frame, start_time, start);

  l.waitForTransform (observation_frame, tracking_frame, end_time, ros::Duration(0.5));
  l.lookupTransform(observation_frame, tracking_frame, end_time, end);


  tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
  tfScalar ang = quat_temp.getAngle();
  
  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


  tf::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(), 
   (delta_y)/corrected_averaging_interval.toSec(),
   (delta_z)/corrected_averaging_interval.toSec());
  tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


  
  tf::StampedTransform inverse;
  l.waitForTransform (reference_frame,tracking_frame,  target_time, ros::Duration(0.5));
  l.lookupTransform(reference_frame,tracking_frame,  target_time, inverse);
  tf::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);

  twist.linear.x =  out_vel.x();
  twist.linear.y =  out_vel.y();
  twist.linear.z =  out_vel.z();
  twist.angular.x =  out_rot.x();
  twist.angular.y =  out_rot.y();
  twist.angular.z =  out_rot.z();

};






  void copyvect(double* a,const std::initializer_list<double>& b)
  { 
    for (int i=0;i< b.size();i++) {
      a[i] = *(b.begin()+i);};
    };



    void initialCMD(jcmd& cmdR,jcmd& cmdL){
      cmdL.names.clear();
      cmdR.names.clear();
      cmdL.names.push_back("left_e0");
      cmdL.names.push_back("left_e1");
      cmdL.names.push_back("left_s0");
      cmdL.names.push_back("left_s1");

      cmdL.names.push_back("left_w0");
      cmdL.names.push_back("left_w1");
      cmdL.names.push_back("left_w2");
    // command joints in the order shown in baxter_interface
      cmdR.names.push_back("right_e0");
      cmdR.names.push_back("right_e1");
      cmdR.names.push_back("right_s0");
      cmdR.names.push_back("right_s1");

      cmdR.names.push_back("right_w0");
      cmdR.names.push_back("right_w1");
      cmdR.names.push_back("right_w2");
  // set your calculated velocities
      cmdL.command.resize(cmdL.names.size());
      cmdR.command.resize(cmdR.names.size());

      copyvect( &cmdL.command[0],{0,90*3.14/180,-0.7854,0,0,0,0});
      copyvect(&cmdR.command[0],{ 0,90*3.14/180,0.7854,0,0,0,0});


      cmdL.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
      cmdR.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    };



    void printChain(KDL::Chain chain){
     vector<KDL::Segment> s=chain.segments;
     vector<KDL::Segment>::iterator iter;
     for (iter = s.begin(); iter != s.end(); iter++) {
   cout << iter->getName() << "\n";
   cout <<"J  =>"<< iter->getJoint().getName() << "\n";
     };

   };


   class Kinematic_controller
   {
   private:

   public:
    ros::NodeHandle n_ ;
// KDL members
    KDL::Tree tree_;

    KDL::Chain limbL_, limbR_;

    KDL::ChainJntToJacSolver *jsL_ , *jsR_ ;

    KDL::JntArray ql_, qr_;

    KDL::Jacobian JL_,JR_;

    tf::TransformListener listener_ ;
    tf::Transformer t;
    ros::Subscriber joint_states_sub;
    sensor_msgs::JointState joint_state;
    ros::Publisher pub_rate ;
    ros::Publisher pub_joint_cmd ;
    ros::Publisher pub_joint_cmd_timeout;
    ros::Publisher  pub_speed_ratio;
    ros::Publisher qdot_left_pub;
    ros::Publisher qdot_right_pub ;

 
   

  
    arma::vec desiredQDotR;
    arma::vec desiredQDotL;
    jcmd cmdL, cmdR;
	// This is the class constructor function
    Kinematic_controller(KDL::Tree& mytree_){ 
    //Topic you want to publish
      qdot_left_pub = n_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 100);
      qdot_right_pub = n_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 100);
      pub_rate = n_.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);
      pub_speed_ratio = n_.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
      pub_joint_cmd = n_.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
      pub_joint_cmd_timeout = n_.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);
    //Topic you want to subscribe
      joint_states_sub= n_.subscribe("/robot/joint_states", 100, &Kinematic_controller::callback_joints, this);

    //Fill tree_ member
      tree_=mytree_;

    // Get single chain from tree_
      tree_.getChain ("base","left_hand", limbL_);
      printChain(limbL_);
      tree_.getChain ("base","right_hand", limbR_);


    // Create empty Jacobians for the two limbs
      JL_=KDL::Jacobian( limbL_.getNrOfJoints() );
      JR_=KDL::Jacobian( limbR_.getNrOfJoints() );


    // Create empty  joint angle vectors
      ql_=KDL::JntArray( limbL_.getNrOfJoints() );
      qr_=KDL::JntArray( limbR_.getNrOfJoints() );

	 // Create jacobian solvers
      jsL_=new KDL::ChainJntToJacSolver(limbL_);
      jsR_=new KDL::ChainJntToJacSolver(limbR_);

  // Target joint velocities

      desiredQDotR=arma::vec(7);
      desiredQDotL=arma::vec(7);

    };

    void callback_joints(const sensor_msgs::JointState::ConstPtr& msg){



     static ros::Time begin=ros::Time::now();
     static double K=1.0;
     double Psi;
     ros::Duration delta_t = ros::Time::now() -begin;
     double delta_t_sec = delta_t.toSec();


     ros::Duration d(2,0);

    Psi=K;//*(delta_t_sec-d.toSec())/(d.toSec());
    if (delta_t_sec>d.toSec()){
     K=-K;
     begin=ros::Time::now();};

   
   
   joint_state.name.resize(19);
   joint_state.position.resize(19);

        //controllare che anche il baxter vero pubblichi questi campi
   joint_state.name= {"head_pan", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint", "right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"};
   for(int i=0;i<19;i++){
    joint_state.position[i] = msg->position[i];
    cout<<endl<<i<<endl<<msg->position[i]<<endl;};



  ql_.data<<joint_state.position[6-1],
  joint_state.position[7-1],
  joint_state.position[4-1],
  joint_state.position[5-1],
  joint_state.position[8-1],
  joint_state.position[9-1],
  joint_state.position[10-1];

  qr_.data<<joint_state.position[15-1],
  joint_state.position[16-1],
  joint_state.position[13-1],
  joint_state.position[14-1],
  joint_state.position[17-1],
  joint_state.position[18-1],
  joint_state.position[19-1];
  cout<<"qr_"<<endl<<qr_.data<<endl;
  cout<<"ql_"<<endl<<ql_.data;
 //attendi();
  
  KDL::ChainFkSolverPos_recursive FKSL(limbL_);
  KDL::ChainFkSolverPos_recursive FKSR(limbR_);
  KDL::Frame torso2wR,torso2wL;

  jsL_->JntToJac(ql_,JR_) ;
  jsR_->JntToJac(qr_,JL_) ;
  FKSL.JntToCart(ql_,torso2wL);
  FKSR.JntToCart(qr_,torso2wR);
  cout<<endl<<"JL"<<endl<<endl<<JL_.data<<endl;


  arma::mat JR(6,7,arma::fill::zeros),JL(6,7,arma::fill::zeros);
  arma::mat pinvJR(7,6,arma::fill::zeros),pinvJL(7,6,arma::fill::zeros);


  for(int i=0;i<6;i++)
    for(int j=0;j<7;j++)
      JR(i,j)=JR_.data(i,j);


    for(int i=0;i<6;i++)
      for(int j=0;j<7;j++)
        JL(i,j)=JL_.data(i,j);

      pinvJR=arma::pinv(JR);
      pinvJL=arma::pinv(JL);

      cout<<"JL"<<endl<<JL<<endl;
      //attendi();



      arma::vec twistL(6,arma::fill::zeros), twistR(6,arma::fill::zeros);

        double r,p,y;
        double X=torso2wL.p[0];
        double Y=torso2wL.p[1];
        double Z=torso2wL.p[2];
        torso2wL.M.GetRPY(r,p,y);  
//Old values
        

desiredQDotR<<0<<endr<<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr;
       desiredQDotL<<0<<endr<<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr
      <<0<<endr;
static double I=0;
static ros::Time to=ros::Time::now();
double dt=ros::Time::now().toSec()-to.toSec();
to=ros::Time::now();
Psi=0;
I=I+(X-0.8)*dt;
twistL<<0.0*Psi<<endr
      <<0.1*Psi<<endr
      <<0.1*Psi<<endr
      <<0<<endr
      <<0<<endr
      <<1<<endr;

      twistR<<0*Psi<<endr
      <<-0.1*Psi<<endr
      <<0.1*Psi<<endr
      <<0<<endr
      <<0<<endr
      <<1<<endr;
cout<<"X"<<X<<endl;

          desiredQDotR=pinvJR*(twistR)+JR.t()*(twistR);
          desiredQDotL=pinvJL*(twistL)+JL.t()*(twistR);

          cmdL.names.clear();
          cmdR.names.clear();

          cmdL.names.push_back("left_s0");
          cmdL.names.push_back("left_s1");
          cmdL.names.push_back("left_e0");
          cmdL.names.push_back("left_e1");
          cmdL.names.push_back("left_w0");
          cmdL.names.push_back("left_w1");
          cmdL.names.push_back("left_w2");
    // command joints in the order shown in baxter_interface

          cmdR.names.push_back("right_s0");
          cmdR.names.push_back("right_s1");
          cmdR.names.push_back("right_e0");
          cmdR.names.push_back("right_e1");
          cmdR.names.push_back("right_w0");
          cmdR.names.push_back("right_w1");
          cmdR.names.push_back("right_w2");
  // set your calculated velocities
          cmdL.command.resize(cmdL.names.size());
          cmdR.command.resize(cmdR.names.size());
    //cout<<cmdL.names.size()<<endl;
          for(size_t i = 0; i < cmdL.names.size(); i++){
    /* //cout<<i<<endl;*/
           cmdL.command[i] = desiredQDotL(i);
           cmdR.command[i] = desiredQDotR(i);
           cmdL.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
           cmdR.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
           cout<<cmdR<<endl;
           cout<<"nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn";
          //attendi();
         }


       }

     


   };
//End of class 


   int main(int argc, char **argv)
   {       
  //Initiate ROS
    ros::init(argc, argv, "Kinematic_controller");

    bool parsed;
    KDL::Tree mytree_;
    parsed=kdl_parser::treeFromFile("/home/giorgio/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", mytree_);


    if (!parsed){
      ROS_ERROR("Failed to construct kdl tree_");
      return false;
    }
    
      KDL::SegmentMap sm;
      sm=mytree_.getSegments();
      Kinematic_controller kin(mytree_);

      ros::Rate rate(100);


      ros::WallDuration e(4,0);

      jcmd cmdinitR,cmdinitL;
      initialCMD(cmdinitR,cmdinitL);
      const ros::WallTime start_time=ros::WallTime::now();




      while(kin.n_.ok()){

        ros::WallTime now=ros::WallTime::now();

        ros::WallDuration delta_t = now- start_time;


cout<<"delta_T"<<delta_t.toSec()<<"e"<<e.toSec()<<endl;
        if (delta_t.toSec()>e.toSec()){

cout<<"||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"<<endl;
         kin.qdot_left_pub.publish(kin.cmdL);
         kin.qdot_right_pub.publish(kin.cmdR);
         cout<<(kin.cmdL);


         ros::Time mytime=ros::Time::now();
         mytime.sec--;
         mytime.sec--;

         tf::StampedTransform transform;
         try{
           kin.listener_.lookupTransform("/left_hand", "/right_hand",mytime, transform);
         }
         catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        };
        cout<<transform.getOrigin().y()<<endl;

        const ros::Duration  averaging_interval(0.005);


      }
      else
   {
 kin.qdot_left_pub.publish(cmdinitL);
 kin.qdot_right_pub.publish(cmdinitR);
};

/*

geometry_msgs::Twist handtwistL; 


try{
  mylookupTwist( kin.listener_,"/left_hand", "/base","/base",ros::Time(), ros::Duration(0.1), handtwistL) ;
}
catch (tf::TransformException ex){
 ROS_ERROR("%s",ex.what());
};

cout<< handtwistL <<endl;*/
ros::spinOnce();
rate.sleep();
};

kin.qdot_left_pub.publish(cmdinitL);
kin.qdot_right_pub.publish(cmdinitR);




return 0;
}




