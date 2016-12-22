#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class OdomCorrigido
{
private:
	ros::NodeHandle nh;
	ros::Subscriber jntSub;
	ros::Subscriber cvSub;
	ros::Publisher odomPub;
	ros::Publisher sPosePub; // Publisher de mensagem do tipo pose stamped.
	tf::TransformBroadcaster odom_broadcaster; // 
	double x, y, theta;
	int countCorrecao, countJointState;
public:
	OdomCorrigido();
	void jointCallback(const sensor_msgs::JointState::ConstPtr&  msg);
	void cvCallback(const std_msgs::Float64MultiArray::ConstPtr& corr);
};

OdomCorrigido::OdomCorrigido()
{
	//inicialização das variáveis privadas:
	x = 5.5119e-02;
	y = -4.8883e-04;
	theta = 1.0000e-02;
	countCorrecao = 0;
	countJointState = 1;

	//Callbacks
	cvSub = nh.subscribe("/CorrecaoPose", 1,&OdomCorrigido::cvCallback, this);
	jntSub = nh.subscribe("/JointState", 1,&OdomCorrigido::jointCallback, this);
	odomPub = nh.advertise<nav_msgs::Odometry>("Odometria", 10);
	sPosePub = nh.advertise<geometry_msgs::PoseStamped>("OdomToVrep",10);
}

// Correcao de odometria:
void OdomCorrigido::cvCallback(const std_msgs::Float64MultiArray::ConstPtr& corr) 
{
	if(corr->data.size() >= 1)
	{
		x = corr->data[0];
		y = corr->data[1];
		theta = corr->data[2];
		countCorrecao++;
		//cout<< "\nPosicoes corrigidas\n";
	}
}

// Cálculo da odometria:
void OdomCorrigido::jointCallback(const sensor_msgs::JointState::ConstPtr&  msg)
{

	double lPos, rPos, s, b = 0.25408, sl, sr, v, vx, vy;
	if (countCorrecao != countJointState)
	{
		lPos = msg->velocity[2];
		rPos = msg->velocity[3];
	
		sr = 0.0425*rPos*0.05;
		sl = 0.0425*lPos*0.05;
		s = ((sr + sl))/2;
		v = s/0.05;

		theta = ((sr - sl))/2*b+ theta;
		x = x + s*cos(theta);
		y = y + s*sin(theta);

		vx = v*cos(theta);
		vy = v*sin(theta);

	}
	else
	{
		cout<< "\n\n Hulll\n";
		countJointState++;
	}
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	
	//tf
	geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

  
    odom_broadcaster.sendTransform(odom_trans);

    //Pose Stamped para o Vrep:
    geometry_msgs::PoseStamped pstamp;
    pstamp.header.stamp = msg->header.stamp;
    pstamp.header.frame_id = "base_link";
    pstamp.pose.position.x = x;
    pstamp.pose.position.y = y;
    pstamp.pose.position.z = 0.0;
    pstamp.pose.orientation = odom_quat;


    //Odom
	nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0.0;

    sPosePub.publish(pstamp);
    odomPub.publish(odom);

	cout<<"X: "<<x<<" Y: "<<y<<" Theta: "<<theta<<endl;
}


int main(int argc, char **argv)
{

	//Inicialização do Node
	ros::init(argc, argv, "odometria_node");
	OdomCorrigido vaique;
	ros::spin();
}
