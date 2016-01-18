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


  // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)


  //correct for the position of the reference frame
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



/*#include <Eigen/SVD>

template<typename _Matrix_Type_>
bool pinv(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
  if(a.rows()<a.cols())
      return false;

  Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd();

  typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

  result = svd.matrixV() * _Matrix_Type_(_Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
      array().inverse(), 0) ).diagonal()) * svd.matrixU().adjoint();
};

bool pinv(const Eigen::Matrix<double,6,7> &a,  Eigen::Matrix<double,7,6> &result, double epsilon = std::numeric_limits< Eigen::Matrix<double,6,7>::Scalar>::epsilon());

*/
typedef baxter_core_msgs::JointCommand jcmd;


template<int r, int c>
Eigen::Matrix<double, c,r>  lev(Eigen::Matrix<double, r, c> M,double lambda)
{

  auto MtM=M.transpose()*M;
    //cout<<MtM<<endl; 
    auto invMtM=MtM.inverse();/*
//cout<<invMtM<<endl;
Eigen::Matrix<double, nc, nr> Mpinv;*/
    auto Mpinv=invMtM*(M.transpose()+lambda*MtM*M.transpose());
    return Mpinv;
  }
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

      copyvect( &cmdL.command[0],{3.14,90*3.14/180,-0.7854,0,0,0,0});
      copyvect(&cmdR.command[0],{ 3.14,90*3.14/180,0.7854,0,0,0,0});


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
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::Transformer t;
    ros::Subscriber joint_states_sub;
    sensor_msgs::JointState joint_state;
    ros::Publisher pub_rate ;
    ros::Publisher pub_joint_cmd ;
    ros::Publisher pub_joint_cmd_timeout;
    ros::Publisher  pub_speed_ratio;
    ros::Publisher qdot_left_pub;
    ros::Publisher qdot_right_pub ;
    KDL::Tree tree;
    KDL::Jacobian jacobianL;
    KDL::Jacobian jacobianR;
    KDL::JntArray ql;
    KDL::JntArray qr;
    KDL::ChainJntToJacSolver* jsL;
    KDL::ChainJntToJacSolver* jsR;
    KDL::Chain limbL;
    KDL::Chain limbR;
    arma::vec desiredQDotR;
    arma::vec desiredQDotL;
    jcmd cmdL, cmdR;
	// This is the class constructor function
    Kinematic_controller(KDL::Tree& mytree)
    { 
    //Topic you want to publish
      qdot_left_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 100);
      qdot_right_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 100);
      pub_rate = n.advertise<std_msgs::UInt16>("robot/joint_state_publish_rate", 10);
      pub_speed_ratio = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 10);
      pub_joint_cmd = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 10);
      pub_joint_cmd_timeout = n.advertise<std_msgs::Float64>("robot/limb/left/joint_command_timeout", 10);
    //Topic you want to subscribe
      joint_states_sub= n.subscribe("/robot/joint_states", 100, &Kinematic_controller::callback_joints, this);

    //Fill tree member
      tree=mytree;

    // Get single chain from tree
      tree.getChain ("base","left_hand", limbL);
      printChain(limbL);
      tree.getChain ("base","right_hand", limbR);

/*attendi(()*/
    // Create empty Jacobians for the two limbs
      jacobianL=KDL::Jacobian( limbL.getNrOfJoints() );
      jacobianR=KDL::Jacobian( limbR.getNrOfJoints() );

      cout<<jacobianL.data;
      //attendi();
    ////cout<<jacobianR.data;
    // Create empty Jacobians for the two limbs
      ql=KDL::JntArray( limbL.getNrOfJoints() );
      qr=KDL::JntArray( limbR.getNrOfJoints() );

	// Create jacobian solvers
      jsL=new KDL::ChainJntToJacSolver(limbL);
      jsR=new KDL::ChainJntToJacSolver(limbR);


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
   for(int i=0;i<19;i++)
    joint_state.position[i] = msg->position[i];



  ql.data<<joint_state.position[6],
  joint_state.position[7],
  joint_state.position[4],
  joint_state.position[5],
  joint_state.position[8],
  joint_state.position[9],
  joint_state.position[10];

  qr.data<<joint_state.position[15],
  joint_state.position[16],
  joint_state.position[13],
  joint_state.position[14],
  joint_state.position[17],
  joint_state.position[18],
  joint_state.position[19];
  cout<<"qr"<<endl<<qr.data<<endl;
  cout<<"ql"<<endl<<ql.data;
 //attendi();
  
  KDL::ChainFkSolverPos_recursive FKSL(limbL);
  KDL::ChainFkSolverPos_recursive FKSR(limbR);
  KDL::Frame torso2wR,torso2wL;

  jsL->JntToJac(ql,jacobianR) ;
  jsR->JntToJac(qr,jacobianL) ;
  FKSL.JntToCart(ql,torso2wL);
  FKSR.JntToCart(qr,torso2wR);
  Eigen::Matrix4d eigen_torso2wL, eigen_torso2wR;


  arma::mat JR(6,7,arma::fill::zeros),JL(6,7,arma::fill::zeros);
  arma::mat pinvJR(7,6,arma::fill::zeros),pinvJL(7,6,arma::fill::zeros);


  for(int i=0;i<6;i++)
    for(int j=0;j<7;j++)
      JR(i,j)=jacobianR.data(i,j);


    for(int i=0;i<6;i++)
      for(int j=0;j<7;j++)
        JL(i,j)=jacobianL.data(i,j);

      pinvJR=arma::pinv(JR);
      pinvJL=arma::pinv(JL);

      cout<<"JR"<<endl<<JL<<endl;
      attendi();
 


      KDL::Twist desiredTwist,twistL,twistR;

      copyvect(desiredTwist.vel.data,{0,0,0});

      copyvect(desiredTwist.rot.data,{0,Psi*5,0});

    twistL=desiredTwist;



    twistR=desiredTwist;

      arma::vec etwistL(6,arma::fill::zeros),etwistR(6,arma::fill::zeros);


      for(int i=0;i<6;i++){
        etwistL(i)=twistL(i);
        etwistR(i)=twistR(i);};

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

          //desiredQDotR=JR.t()*(etwistR);
          //desiredQDotL=JL.t()*(etwistR);

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
    KDL::Tree mytree;
    parsed=kdl_parser::treeFromFile("/home/giorgio/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", mytree);


    if (!parsed){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    
      KDL::SegmentMap sm;
      sm=mytree.getSegments();
      Kinematic_controller kin(mytree);

      ros::Rate rate(100);


      ros::WallDuration e(4,0);

      jcmd cmdinitR,cmdinitL;
      initialCMD(cmdinitR,cmdinitL);
      const ros::WallTime start_time=ros::WallTime::now();




      while(kin.n.ok()){

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
           kin.listener.lookupTransform("/left_hand", "/right_hand",mytime, transform);
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
  mylookupTwist( kin.listener,"/left_hand", "/base","/base",ros::Time(), ros::Duration(0.1), handtwistL) ;
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




