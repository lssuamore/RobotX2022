#ifndef HELLO_GUI_H
#define HELLO_GUI_H

#include <QWidget>
#include <QColor>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
//Heartbeat Code Includes and Constants
#include <arpa/inet.h> // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> // bzero()
#include <sys/socket.h>
#include <unistd.h> // read(), write(), close()
#define MAX 256
#define SA struct sockaddr
#define PORT 8080
#define IP "127.0.0.1"
#define MSG "RXHRB,111221,161229,21.31198,N,157.88972,W,ROBOT,2,2"

namespace Ui {
class HelloGui;
}


class HelloGui : public QWidget
{
  Q_OBJECT

public:
  explicit HelloGui(QWidget *parent = nullptr);
  ~HelloGui();

// Second step to setup subscribers for topics

  // Below lines are for USV
  void USV_latitude(const std_msgs::String::ConstPtr& USV_lat);
  void USV_longitude(const std_msgs::String::ConstPtr& USV_lon);
  void x_usv_NED(const std_msgs::String::ConstPtr& USV_X_NED);
  void y_usv_NED(const std_msgs::String::ConstPtr& USV_Y_NED);
  void psiNED(const std_msgs::String::ConstPtr& NED_psi_msg);
  void USV_mode(const std_msgs::String::ConstPtr& USV_mod);
  void USV_Curr_Obj(const std_msgs::String::ConstPtr& USV_curr);

  // Below lines are for UAV
  void UAV_latitude(const std_msgs::String::ConstPtr& UAV_lat);
  void UAV_longitude(const std_msgs::String::ConstPtr& UAV_lon);
  void x_uav_ENU(const std_msgs::String::ConstPtr& UAV_X_ENU);
  void y_uav_ENU(const std_msgs::String::ConstPtr& UAV_Y_ENU);
  void z_uav_ENU(const std_msgs::String::ConstPtr& UAV_Z_ENU);
  void UAV_Pozyx_X(const std_msgs::String::ConstPtr& UAV_Pozyx_x);
  void UAV_Pozyx_Y(const std_msgs::String::ConstPtr& UAV_Pozyx_y);
  void UAV_Pozyx_Z(const std_msgs::String::ConstPtr& UAV_Pozyx_z);
  void psiENU(const std_msgs::String::ConstPtr& ENU_psi_msg);
  void UAV_mode(const std_msgs::String::ConstPtr& UAV_mod);

  // Below lines are for AMS
  void rcolor(const std_msgs::String::ConstPtr& recolor);
  void gcolor(const std_msgs::String::ConstPtr& grcolor);
  void bcolor(const std_msgs::String::ConstPtr& blcolor);
  void Color_Light (const std_msgs::String::ConstPtr& light);
  void Animal_Color_Light (const std_msgs::String::ConstPtr& putrycolors);
//  void Tech_Sever_Ip(const std_msgs::String::ConstPtr& server);
//  void Team_INfo(const std_msgs::String::ConstPtr& info);
//  void PORt(const std_msgs::String::ConstPtr& por);


  void heartbeat();
public slots:
    void spinOnce();


//private slots:
//    void on_Server_IP_Input_Box_released();

private:
    Ui::HelloGui *ui;
    QString *ip;
    QTimer *ros_timer;


    ros::NodeHandlePtr nh_;

// First step for the subscribers from the talker node

    // USV
    ros::Subscriber USV_latitude_sub_;
    ros::Subscriber USV_longitude_sub_;
    ros::Subscriber x_usv_NED_sub_;
    ros::Subscriber y_usv_NED_sub_;
    ros::Subscriber psiNED_str_sub_;
    ros::Subscriber USV_mode_sub_;
    ros::Subscriber USV_Curr_Obj_sub_;


    // UAV
    ros::Subscriber UAV_latitude_sub_;
    ros::Subscriber UAV_longitude_sub_;
    ros::Subscriber x_uav_ENU_str_sub_;
    ros::Subscriber y_uav_ENU_str_sub_;
    ros::Subscriber z_uav_ENU_str_sub_;
    ros::Subscriber UAV_Pozyx_X_str_sub_;
    ros::Subscriber UAV_Pozyx_Y_str_sub_;
    ros::Subscriber UAV_Pozyx_Z_str_sub_;
    ros::Subscriber psiENU_str_sub_;
    ros::Subscriber UAV_Mode_sub_;
    ros::Subscriber UAV_heading_sub_;

    // AMS
    ros::Subscriber Color_Light_sub_;
    ros::Subscriber Animal_Color_Light_sub_;
//    ros::Subscriber nav_NED_sub_;
    ros::Subscriber imu_sub;
    ros::Subscriber obj_rec_sub_;
    ros::Subscriber CC_animals_ned_sub_;
    // To send out info below if added in correct subscribers above
//    ros::Publisher Server_IP_pub_;
//    ros::Publisher Team_INfo_pub_;
//    ros::Publisher PORt_pub_;
//    ros::Subscriber Tech_Sever_Ip_sub_;

};


//========================================== Heartbeat code below ================================================
/*open a tcp socket and send an NMEA string*/
class heartbeat_sock
{
private:
    char ip[MAX];
    char port[MAX];
    int sockfd, connfd;
public:
    //open socket  ip (v4 format), port
  heartbeat_sock(const char [], const int);
    //closes socket
  ~heartbeat_sock();
    //send NMEA string packet; sockfd, message to send
    int send(const char *);
};

  #endif // HELLO_GUI_H
