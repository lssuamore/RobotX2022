// Talker subscribes and publishes information in the correct data type


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/NavSatFix.h"		// Place where the lat and long for the USV is read from
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/PoseStamped.h>
#define PI 3.14159265


//Task 4
std::string Animal, Animal_Temp;  // string array to hold the names of the animals
char Animal_str = {}; // String to send animals for heartbeat
int animal_qty = 0;

//Task 5
std::string Object;  // string array to hold the names of the objects from the zed2i
float x_object_NED[9], y_object_NED[9];  // arrays to hold animal locations
std::string Cod_Seq, Cod_Seq_Temp = "Nothing";
int cnt = 0;
int black = 0;




//========================================Other Variables========================================
int redcolor,greencolor,bluecolor;



//=========================================USV Variables=========================================
float latitude_USV,longitude_USV,altitude_USV, x_NED_USV, y_NED_USV;
float q1_usv, q2_usv, q3_usv, q0_usv;	// Sparton imu quarternion
float phi_usv_ENU, theta_usv_ENU, psi_usv_ENU;	// conversion to radians
float omega_x_usv, omega_y_usv, omega_z_usv;
float q1, q2, q3, q0;	// Sparton imu quarternion
float phi, theta, psi;	// conversion to radians
float phiNED, thetaNED, psiNED;
float omega_x, omega_y, omega_z;
float USV_local_X, USV_local_Y, USV_local_Z;
int USV_state_str;
std_msgs::String Curr_Task;



//=========================================UAV Variables=========================================
float latitude_UAV,longitude_UAV,altitude_UAV, x_uav_ENU, y_uav_ENU,z_uav_ENU;
float UAV_Pozyx_X, UAV_Pozyx_Y, UAV_Pozyx_Z;
//float phiNED, thetaNED, psiNED;
float q1_uav, q2_uav, q3_uav, q0_uav;  //	IMU orientation for UAV
float phi_uav_ENU, theta_uav_ENU, psi_uav_ENU;  // Conversion to radians for UAV
float omega_x_uav, omega_y_uav, omega_z_uav;  // Angular velocities
int UAV_state_str;



//=========================================AMS Functions=========================================

// The function below is from code already written
// The voids are all functions used to calculate and change data types
void object_rec(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  int i = 0;
       Object = msg->header.frame_id;  // Getting array of animal name. Based on on array location, the object is closer or farther from USV

  if (Object == "Blue Light")
  {
    Cod_Seq = "Blue";
    black = 0;
    cnt++;
  }
  else if (Object == "Red Light")
  {
    Cod_Seq = "Red";
    black = 0;
    cnt++;
  }
  else if (Object == "Green Light")
  {
    Cod_Seq= "Green";
    black = 0;
    cnt++;
  }
  else if (Object == "Black Light")
  {
    black++;
  }
} // END OF object_rec()

// Amount of animals seen to put only 1 in each box
// Put the correct animal seen into the correct box
void CC_animals_ned_update(const geometry_msgs::PointStamped::ConstPtr& object)
{
    animal_qty = object->header.seq;
    //Animal_str = animal_qty + '0';
    Animal = object->header.frame_id;  // Getting array of animal names
}  // END OF CC_animals_ned_update()



//=========================================USV Functions=========================================
// USV x and y location
void pose_update(const nav_msgs::Odometry::ConstPtr& odom){
  x_NED_USV = odom->pose.pose.position.x;
  y_NED_USV = odom->pose.pose.position.y;
}


// USV heading
void imu_processor(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
 // gather orientation quaternion
 q1 = imu_msg->orientation.x;
 q2 = imu_msg->orientation.y;
 q3 = imu_msg->orientation.z;
 q0 = imu_msg->orientation.w;

 // gather body-fixed angular velocity
 omega_x = imu_msg->angular_velocity.x;
 omega_y = imu_msg->angular_velocity.y;
 omega_z = imu_msg->angular_velocity.z;

 //converts to radians
 psi = atan2((2.0*(q3*q0 + q1*q2)) , (1.0 - 2.0*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
 // Convert the orientation to NED from ENU
 psiNED = -psi -0.525;    // Heading

 while ((psiNED < -PI) || (psiNED > PI))
 {
   ROS_INFO("Entering while after atan2, understandable");
   // Adjust psiNED back within -PI and PI
   if (psiNED < -PI)
   {
     psiNED = psiNED + 2.0*PI;
   }
   if (psiNED > PI)
   {
     psiNED = psiNED - 2.0*PI;
   }
 }
}

// USV mode
void USV_status(const std_msgs::Int32::ConstPtr& msg)
{
  USV_state_str = msg->data;
}

// Current task of the USV
void Current_Task(const std_msgs::String::ConstPtr& Task_msg)
{
  Curr_Task.data = Task_msg->data;
}

// USV latitidue and longitude from the NavSatFix message
void gps_processor(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
  latitude_USV = gps_msg->latitude; //sets latitude from gps
  longitude_USV = gps_msg->longitude; //sets longitude from gps
  altitude_USV = gps_msg->altitude; //sets altitude rom gps
} // END OF gps_processor()



//=========================================UAV Functions=========================================
// UAV latitidue and longitude from the NavSatFix message
void UAV_gps(const sensor_msgs::NavSatFix::ConstPtr& uav_msg)
{
  latitude_UAV = uav_msg->latitude; //sets latitude from gps
  longitude_UAV = uav_msg->longitude; //sets longitude from gps
  altitude_UAV = uav_msg->altitude; //sets altitude from gps
}

// UAV heading
void imu_uav(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
 // gather orientation quaternion
 q1_uav = imu_msg->orientation.x;
 q2_uav = imu_msg->orientation.y;
 q3_uav = imu_msg->orientation.z;
 q0_uav = imu_msg->orientation.w;

 // gather body-fixed angular velocity
 omega_x_uav = imu_msg->angular_velocity.x;
 omega_y_uav = imu_msg->angular_velocity.y;
 omega_z_uav = imu_msg->angular_velocity.z;

 //converts to radians
 phi_uav_ENU = atan2((2.0*(q1_uav*q0_uav + q3_uav*q2_uav)) , (1.0 - 2.0*(pow(q1_uav,2.0) + pow(q2_uav,2.0))));
 theta_uav_ENU = asin(2.0*(q0_uav*q2_uav - q1_uav*q3_uav));
 psi_uav_ENU = atan2((2.0*(q3_uav*q0_uav + q1_uav*q2_uav)) , (1.0 - 2.0*(pow(q2_uav,2.0) + pow(q3_uav,2.0)))); // orientation off x-axis
 // Convert the orientation to NED from ENU
 //phiNED = theta;
 //thetaNED = phi;
 //psiNED = -psi -0.525;

 while ((psi_uav_ENU < -PI) || (psi_uav_ENU > PI))
 {
   //ROS_INFO("Entering while after atan2, understandable");
   // Adjust psi_uav_ENU back within -PI and PI
   if (psi_uav_ENU < -PI)
   {
     psi_uav_ENU = psi_uav_ENU + 2.0*PI;
   }
   if (psi_uav_ENU > PI)
   {
     psi_uav_ENU = psi_uav_ENU - 2.0*PI;
   }
 }
}


// UAV mode
void UAV_status(const std_msgs::Int32::ConstPtr& msg)
{
  UAV_state_str = msg->data;
}


// UAV x, y, and z GPS position
void UAV_ENU(const geometry_msgs::PoseStamped::ConstPtr& enu_uav)
{
  // Finding the x,y,z position at that location and setting it to the respective variable
  x_uav_ENU = enu_uav->pose.position.x;
  y_uav_ENU = enu_uav->pose.position.y;
  z_uav_ENU = enu_uav->pose.position.z;
} // END OF ned_func(const nav_msgs::Odometry::ConstPtr& enu_state)


// UAV x, y, and z local position
void gps_position_update(const geometry_msgs::Point::ConstPtr& gps_msg)
{
  UAV_Pozyx_X = gps_msg->x;
  UAV_Pozyx_Y = gps_msg->y;
  UAV_Pozyx_Z = gps_msg->z;
}



//==============================================Main==============================================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n("~");


// The publishers for the variable_name_pub are all below

// Below are USV items
  ros::Publisher USV_latitude_pub = n.advertise<std_msgs::String>("USV_latitude", 1);
  ros::Publisher USV_longitude_pub = n.advertise<std_msgs::String>("USV_longitude", 1);
  ros::Publisher x_usv_NED_pub = n.advertise<std_msgs::String>("x_usv_NED", 1);
  ros::Publisher y_usv_NED_pub = n.advertise<std_msgs::String>("y_usv_NED", 1);
  ros::Publisher psiNED_str_pub = n.advertise<std_msgs::String>("psiNED_str", 1);
  ros::Publisher USV_Mode_pub = n.advertise<std_msgs::String>("USV_mode", 1);
  ros::Publisher USV_Current_str_pub = n.advertise<std_msgs::String>("USV_Current", 1);


// Below are UAV items
  ros::Publisher UAV_latitude_pub = n.advertise<std_msgs::String>("UAV_latitude", 1);
  ros::Publisher UAV_longitude_pub = n.advertise<std_msgs::String>("UAV_longitude", 1);
  ros::Publisher x_uav_ENU_str_pub = n.advertise<std_msgs::String>("x_uav_ENU_str", 1);
  ros::Publisher y_uav_ENU_str_pub = n.advertise<std_msgs::String>("y_uav_ENU_str", 1);
  ros::Publisher z_uav_ENU_str_pub = n.advertise<std_msgs::String>("z_uav_ENU_str", 1);
  ros::Publisher UAV_Mode_pub = n.advertise<std_msgs::String>("UAV_mode", 1);
  ros::Publisher UAV_Pozyx_X_str_pub = n.advertise<std_msgs::String>("UAV_Pozyx_X_str", 1);
  ros::Publisher UAV_Pozyx_Y_str_pub = n.advertise<std_msgs::String>("UAV_Pozyx_Y_str", 1);
  ros::Publisher UAV_Pozyx_Z_str_pub = n.advertise<std_msgs::String>("UAV_Pozyx_Z_str", 1);
  ros::Publisher psiENU_str_pub = n.advertise<std_msgs::String>("psiENU_str", 1);


// Below are AMS/heartbeat items
  ros::Publisher LightStack_pub = n.advertise<std_msgs::String>("Color_Light", 1);
  ros::Publisher Animal_Color_pub = n.advertise<std_msgs::String>("Animal_Color_Light", 1);
// Needed only if the topics are added
//  ros::Publisher Server_IP_pub = n.advertise<std_msgs::String>("Tech_Sever_Ip", 1);
//  ros::Publisher Team_INfo_pub = n.advertise<std_msgs::String>("Team_INfo", 1);
//  ros::Publisher PORt_pub = n.advertise<std_msgs::String>("PORt", 1);


// To make the subscribers for the variable_name_sub are all below

// Below are USV items
  ros::Subscriber gpspos_sub = n.subscribe("/gps/fix", 1, gps_processor);  // Subscribes to GPS position from outdoor gps
  ros::Subscriber nav_NED_sub = n.subscribe("/NA_nav_ned", 1, pose_update);
  ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imu_processor);	// subscribes to IMU
  ros::Subscriber USV_mode_sub = n.subscribe("/USV_Mode", 1, USV_status);
  ros::Subscriber Current_Task_sub = n.subscribe("/USV_Task", 1, Current_Task);


// Below are AMS items
  ros::Subscriber obj_rec_sub = n.subscribe("/obj_rec", 10, object_rec);  // Obtains what object the camera recognise, pose of the object and order of recognition
  ros::Subscriber CC_animals_ned_sub = n.subscribe("/CC_animals_ned", 10, CC_animals_ned_update );


// Below are UAV items
  ros::Subscriber UAV_Mode_sub = n.subscribe("/UAV_Mode", 1, UAV_status);
  ros::Subscriber uav_gps_sub = n.subscribe("/mavros/global_position/global", 10, UAV_gps);
  ros::Subscriber gps_position_update_sub = n.subscribe("/point", 1, gps_position_update);
  ros::Subscriber enu_gps = n.subscribe("/mavros/local_position/pose", 1, UAV_ENU);
  ros::Subscriber UAV_imu_sub = n.subscribe("/mavros/imu/data", 1, imu_uav);	// subscribes to IMU
//  ros::Subscriber gpspos_sub = n.subscribe("/gps/fix", 1, gps_processor);
//  ros::Subscriber nav_NED_sub = n.subscribe("nav_ned", 1, pose_update);


  // How often the program checks in milliseconds
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
   // Variable names used for string data types
   std::stringstream ss1, ss2, ss3, ss4, ss5, ss6, ss7, ss8, ss9, ss10, ss11, ss12, ss13, ss14, ss15, ss16, ss17, ss18, ss19;


// Below are USV items
    std_msgs::String USV_latitude;
    std_msgs::String USV_longitude;
    std_msgs::String x_usv_NED;
    std_msgs::String y_usv_NED;
    std_msgs::String USV_Mode;
    std_msgs::String psiNED_str;
    std_msgs::String USV_Current_str;


        ss1 << latitude_USV;
        USV_latitude.data = ss1.str();

        ss2 << longitude_USV;
        USV_longitude.data = ss2.str();
// For testing and debugging it will show on the rosrun template_gui_package talker terminal
//        ROS_INFO("lat = %f", latitude_USV);
//        ROS_INFO("psi = %f", psiNED);
//        ROS_INFO("%s", USV_latitude.data.c_str());  // To display the data for debugging
//        ROS_INFO("%s", USV_longitude.data.c_str());

        ss3 << x_NED_USV;
        x_usv_NED.data = ss3.str();

        ss4 << y_NED_USV;
        y_usv_NED.data = ss4.str();

        ss5 << USV_state_str;
        USV_Mode.data = ss5.str();
//         ROS_INFO("%s", Mode.data.c_str());

        ss6 << psiNED;
        psiNED_str.data = ss6.str();
//         ROS_INFO("psiNED_str = %s", psiNED_str.data.c_str());

        ss19 << Curr_Task.data;
        USV_Current_str.data = ss19.str();
//        ROS_INFO("USV_Current_str = %s", USV_Current_str.data.c_str());


        // To publish UAV all the data above
        USV_latitude_pub.publish(USV_latitude);
        USV_longitude_pub.publish(USV_longitude);
        x_usv_NED_pub.publish(x_usv_NED);
        y_usv_NED_pub.publish(y_usv_NED);
        USV_Mode_pub.publish(USV_Mode);
        psiNED_str_pub.publish(psiNED_str);
        USV_Current_str_pub.publish(USV_Current_str);


// Below are UAV items

    // Setting up the variables to the correct data type
    std_msgs::String UAV_latitude;
    std_msgs::String UAV_longitude;
    std_msgs::String x_uav_ENU_str;
    std_msgs::String y_uav_ENU_str;
    std_msgs::String z_uav_ENU_str;
    std_msgs::String UAV_Pozyx_X_str;
    std_msgs::String UAV_Pozyx_Y_str;
    std_msgs::String UAV_Pozyx_Z_str;
    std_msgs::String UAV_Mode;
    std_msgs::String psiENU_str;


    ss7 << latitude_UAV;
    UAV_latitude.data = ss7.str();

    ss8 << longitude_UAV;
    UAV_longitude.data = ss8.str();

    ss9 << x_uav_ENU;
    x_uav_ENU_str.data = ss9.str();

    ss10 << y_uav_ENU;
    y_uav_ENU_str.data = ss10.str();

    ss11 << z_uav_ENU;
    z_uav_ENU_str.data = ss11.str();
// Used for debugging it will show on the rosrun template_gui_package talker terminal
//     ROS_INFO("z_uav_ENU_str = %s", z_uav_ENU_str.data.c_str());

    ss12 << UAV_Pozyx_X;
    UAV_Pozyx_X_str.data = ss12.str();

    ss13 << UAV_Pozyx_Y;
    UAV_Pozyx_Y_str.data = ss13.str();

    ss14 << UAV_Pozyx_Z;
    UAV_Pozyx_Z_str.data = ss14.str();

    ss15 << UAV_state_str;
    UAV_Mode.data = ss15.str();

    ss16 << psi_uav_ENU;
    psiENU_str.data = ss16.str();
//     ROS_INFO("psiENU_str = %s", psiENU_str.data.c_str());


    // To publish UAV all the data above
    UAV_latitude_pub.publish(UAV_latitude);
    UAV_longitude_pub.publish(UAV_longitude);
    x_uav_ENU_str_pub.publish(x_uav_ENU_str);
    y_uav_ENU_str_pub.publish(y_uav_ENU_str);
    z_uav_ENU_str_pub.publish(z_uav_ENU_str);
    UAV_Pozyx_X_str_pub.publish(UAV_Pozyx_X_str);
    UAV_Pozyx_Y_str_pub.publish(UAV_Pozyx_Y_str);
    UAV_Pozyx_Z_str_pub.publish(UAV_Pozyx_Z_str);
    UAV_Mode_pub.publish(UAV_Mode);
    psiENU_str_pub.publish(psiENU_str);

//    ROS_INFO("%s", Mode.data.c_str());


// Below are AMS items

    // Setting up the variables to the correct data type
    std_msgs::String Light;  // Setting Light as a string variable
    std_msgs::String AnimalC;
//  std_msgs::String Mode;

    ss17 << Cod_Seq;
    Light.data = ss17.str();
     if (Cod_Seq != Cod_Seq_Temp)
      {
       LightStack_pub.publish(Light);
       Cod_Seq_Temp = Cod_Seq;
      }

    ss18 << Animal;
    AnimalC.data = ss18.str();

    if (Animal != Animal_Temp)
     {
      Animal_Color_pub.publish(AnimalC);
      Animal_Temp = Animal;
     }


     ros::spinOnce();
     loop_rate.sleep();
  }
  return 0;
}
