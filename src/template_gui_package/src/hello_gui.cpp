// This .cpp file displays data to the .ui file

// Required includes for full function of gui
#include "hello_gui.h"
#include "ui_hello_gui.h"
#include <QDebug>
#include <ros/console.h>
#include "sensor_msgs/NavSatFix.h"  // Place where the lat and long for the USV is read from
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <QColor>  // For the color tasks sets colors(rgb values)
#include <QPalette>  // For setting the boxes background colors


int spot = 1;
//QString server, ipp = "", inf = "", ipo = "";


HelloGui::HelloGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::HelloGui)
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  std::string listen_topic;  // The set up for the string listen_topic

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(10);  // set the rate to 10ms  You can change this if you want to increase/decrease update rat

  // Below are AMS items
  nh_->param<std::string>("listen_topic",listen_topic,"/talker/Color_Light");
  Color_Light_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::Color_Light, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/Animal_Color_Light");
  Animal_Color_Light_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::Animal_Color_Light, this);


//===================================== Below are USV items ======================================================
  // setup subscriber by according to the ~/Chatter_topic param

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/USV_latitude");
  USV_latitude_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::USV_latitude, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/USV_longitude");
  USV_longitude_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::USV_longitude, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/x_usv_NED");
  x_usv_NED_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::x_usv_NED, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/y_usv_NED");
  y_usv_NED_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::y_usv_NED, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/USV_mode");
  USV_mode_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::USV_mode, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/psiNED_str");
  psiNED_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::psiNED, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/USV_Current");
  USV_Curr_Obj_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::USV_Curr_Obj, this);


//===================================== Below are UAV items ======================================================
  // Below are UAV items
  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_latitude");
  UAV_latitude_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_latitude, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_longitude");
  UAV_longitude_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_longitude, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_mode");
  UAV_Mode_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_mode, this);

// The UAV Current Objective
//  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_curr");
//  UAV_Curr_Obj_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_curr, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/psiENU_str");
  psiENU_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::psiENU, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/x_uav_ENU_str");
  x_uav_ENU_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::x_uav_ENU, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/y_uav_ENU_str");
  y_uav_ENU_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::y_uav_ENU, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/z_uav_ENU_str");
  z_uav_ENU_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::z_uav_ENU, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_Pozyx_X_str");
  UAV_Pozyx_X_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_Pozyx_X, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_Pozyx_Y_str");
  UAV_Pozyx_Y_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_Pozyx_Y, this);

  nh_->param<std::string>("listen_topic",listen_topic,"/talker/UAV_Pozyx_Z_str");
  UAV_Pozyx_Z_str_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &HelloGui::UAV_Pozyx_Z, this);

//  // publish a message on the channel specified by ~/hello_topic param
//  std::string hello_topic;
//  nh_->param<std::string>("hello_topic",hello_topic,"Chatter");
//  hello_pub_ = nh_->advertise<std_msgs::String>(hello_topic,1);

//===================================== Below are HeartBeat items ===============================================
// If inputs are needed for heartbeat then the below needs to be uncommented and added

// publish a message on the channel specified by ~/Server_IP param
//  std::string Server_IP;
//  nh_->param<std::string>("Server_IP",Server_IP,"/talker/Tech_Sever_Ip");
//  Server_IP_pub_ = nh_->advertise<std_msgs::String>(Server_IP,1);

//  std::string TEAM_INfo;
//  nh_->param<std::string>("TEAM_Info",TEAM_INfo,"/talker/Team_Info");
//  Team_INfo_pub_ = nh_->advertise<std_msgs::String>(TEAM_INfo,1);

//  std::string PORt;
//  nh_->param<std::string>("PORt",PORt,"/talker/PORt");
//  PORt_pub_ = nh_->advertise<std_msgs::String>(PORt,1);



  heartbeat();
}

HelloGui::~HelloGui()
{
  delete ui;
  delete ros_timer;
}


void HelloGui::spinOnce(){

  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

// The first void is commented and the rest of them follow the same format

// Inputs through the talker node
void HelloGui::USV_latitude(const std_msgs::String::ConstPtr &USV_lat){
// Changes the data type to easily display
    auto USV_latitude_msg = QString::fromStdString( USV_lat->data.c_str() );
// Sets the input to the specified display box
  ui->USV_Lat_Box->setText(USV_latitude_msg);
}

void HelloGui::USV_longitude(const std_msgs::String::ConstPtr &USV_lon){
  auto USV_longitude_msg = QString::fromStdString( USV_lon->data.c_str() );
  ui->USV_Long_Box->setText(USV_longitude_msg);
}

void HelloGui::UAV_latitude(const std_msgs::String::ConstPtr &UAV_lat){
  auto UAV_latitude_msg = QString::fromStdString( UAV_lat->data.c_str() );
  ui->UAV_Lat_Box->setText(UAV_latitude_msg);
}

void HelloGui::UAV_longitude(const std_msgs::String::ConstPtr &UAV_lon){
  auto UAV_longitude_msg = QString::fromStdString( UAV_lon->data.c_str() );
  ui->UAV_Long_Box->setText(UAV_longitude_msg);
}

void HelloGui::x_usv_NED(const std_msgs::String::ConstPtr &USV_X_NED){
  auto x_usv_N_msg = QString::fromStdString( USV_X_NED->data.c_str() );
  ui->USV_X->setText(x_usv_N_msg);
}

void HelloGui::y_usv_NED(const std_msgs::String::ConstPtr &USV_Y_NED){
  auto y_usv_N_msg = QString::fromStdString( USV_Y_NED->data.c_str() );
  ui->USV_Y->setText(y_usv_N_msg);
}

void HelloGui::x_uav_ENU(const std_msgs::String::ConstPtr &UAV_X_ENU){
  auto x_uav_ENU_msg = QString::fromStdString( UAV_X_ENU->data.c_str() );
  ui->UAV_X_GPS->setText(x_uav_ENU_msg);
}

void HelloGui::y_uav_ENU(const std_msgs::String::ConstPtr &UAV_Y_ENU){
  auto y_uav_ENU_msg = QString::fromStdString( UAV_Y_ENU->data.c_str() );
  ui->UAV_Y_GPS->setText(y_uav_ENU_msg);
}

void HelloGui::z_uav_ENU(const std_msgs::String::ConstPtr &UAV_Z_ENU){
  auto z_uav_ENU_msg = QString::fromStdString( UAV_Z_ENU->data.c_str() );
  ui->UAV_Z_GPS->setText(z_uav_ENU_msg);
}

void HelloGui::UAV_Pozyx_X(const std_msgs::String::ConstPtr &UAV_Pozyx_x){
  auto UAV_Pozyx_x_msg = QString::fromStdString( UAV_Pozyx_x->data.c_str() );
  ui->UAV_X_Pozyx->setText(UAV_Pozyx_x_msg);
}

void HelloGui::UAV_Pozyx_Y(const std_msgs::String::ConstPtr &UAV_Pozyx_y){
  auto UAV_Pozyx_y_msg = QString::fromStdString( UAV_Pozyx_y->data.c_str() );
  ui->UAV_Y_Pozyx->setText(UAV_Pozyx_y_msg);
}

void HelloGui::UAV_Pozyx_Z(const std_msgs::String::ConstPtr &UAV_Pozyx_z){
  auto UAV_Pozyx_z_msg = QString::fromStdString( UAV_Pozyx_z->data.c_str() );
  ui->UAV_Z_Pozyx->setText(UAV_Pozyx_z_msg);
}

void HelloGui::USV_mode(const std_msgs::String::ConstPtr &USV_mod){
  auto USV_mod_msg = QString::fromStdString( USV_mod->data.c_str() );
  ui->USV_Mode_Box->setText(USV_mod_msg);  //
}

void HelloGui::UAV_mode(const std_msgs::String::ConstPtr &UAV_mod){
  auto UAV_mod_msg = QString::fromStdString( UAV_mod->data.c_str() );
  ui->UAV_Mode_Box->setText(UAV_mod_msg);  // UAV_mode_msg
}

// For the Current Obj of UAV
void HelloGui::USV_Curr_Obj(const std_msgs::String::ConstPtr &USV_curr){
  auto USV_mode_msg = QString::fromStdString( USV_curr->data.c_str() );
  ui->USV_Current_Obj_Box->setText(USV_mode_msg);
}

void HelloGui::psiNED(const std_msgs::String::ConstPtr& NED_psi_msg){
  auto USV_Heading_msg = QString::fromStdString(NED_psi_msg->data.c_str());
  ui->USV_Heading_Box->setText(USV_Heading_msg);
}

void HelloGui::psiENU(const std_msgs::String::ConstPtr& ENU_psi_msg){
  auto UAV_Heading_msg = QString::fromStdString(ENU_psi_msg->data.c_str());
  ui->UAV_Heading_Box->setText(UAV_Heading_msg);
}


// For the color code sensing program
// The if statements are repeated for each box for each color
QColor Color_Loght_msg;
// As above takes inputs from the talker node
void HelloGui::Color_Light(const std_msgs::String::ConstPtr& light){
// Changes the data type to the correct display type
  auto Color_Light_msg = QString::fromStdString(light->data.c_str());
// Counts through each of the 3 boxes and places the correct color in each box as it counts
  if (Color_Light_msg == "Red")
  {
    Color_Loght_msg.setRgb(255, 0, 0);

    if (spot == 1)
    {
      // Displays the color to the ui
      // the ui points to the box which points to the color
      ui->Spot_One->setText(Color_Light_msg);
      ui->Color_One->setPalette(Color_Loght_msg);
//      QPalette pal = ui->Color_One->palette();
//      pal.setColor(QPalette::Window, Color_Loght_msg);
//      emit Color_Loght_msg;
      spot = 2;
    }
    else if (spot == 2)
    {
      ui->Spot_Two->setText(Color_Light_msg);
      ui->Color_Two->setPalette(Color_Loght_msg);
      spot = 3;
    }
    else if (spot == 3)
    {
      ui->Spot_Three->setText(Color_Light_msg);
      ui->Color_Three->setPalette(Color_Loght_msg);
      spot = 1;
    }
  }
  if (Color_Light_msg == "Green")
  {
    Color_Loght_msg.setRgb(0, 255, 0);
    if (spot == 1)
    {
      ui->Spot_One->setText(Color_Light_msg);
      ui->Color_One->setPalette(Color_Loght_msg);
      spot = 2;
    }
    else if (spot == 2)
    {
      ui->Spot_Two->setText(Color_Light_msg);
      ui->Color_Two->setPalette(Color_Loght_msg);
      spot = 3;
    }
    else if (spot == 3)
    {
      ui->Spot_Three->setText(Color_Light_msg);
      ui->Color_Three->setPalette(Color_Loght_msg);
      spot = 1;
    }
  }
  if (Color_Light_msg == "Blue")
  {
    Color_Loght_msg.setRgb(0, 0, 255);
    if (spot == 1)
    {
      ui->Spot_One->setText(Color_Light_msg);
      ui->Color_One->setPalette(Color_Loght_msg);
      spot = 2;
    }
    else if (spot == 2)
    {
      ui->Spot_Two->setText(Color_Light_msg);
      ui->Color_Two->setPalette(Color_Loght_msg);
      spot = 3;
    }
    else if (spot == 3)
    {
      ui->Spot_Three->setText(Color_Light_msg);
      ui->Color_Three->setPalette(Color_Loght_msg);
      spot = 1;
    }
  }

}

// Task 4
// Below is the code for the animal color reading code
// It is set up very same to the above color code same if statement structure
QColor Animal_Color_Loght_msg;  // Initialization of the color variable
void HelloGui::Animal_Color_Light(const std_msgs::String::ConstPtr& putrycolors){  // Similar to a fuction
  auto Animal_Color_Light_msg = QString::fromStdString(putrycolors->data.c_str());  // Auto data type conversions from string to Qstring
  if (Animal_Color_Light_msg == "crocodile")
  {
    Animal_Color_Light_msg = "crocodile";  // Gives the Qstring variable Animal_Color_Light_msg the correct name to display
    Animal_Color_Loght_msg.setRgb(255, 0, 0);  // Sets the color variable Animal_Color_Loght_msg to Red
    // Spot is the box number that the information below will be displayed in
    if (spot == 1)
    {
      ui->Animal_One->setText(Animal_Color_Light_msg);  // For the ui to display the text in Animal_Color_Light_msg
      ui->Animal_One->setPalette(Animal_Color_Loght_msg);  // For the ui to display the color set by Animal_Color_Loght_msg (Green)
 //     QPalette pal = ui->Animal_One->palette();  // Changes the setPalette variable to a Qpalette data type
 //     pal.setColor(QPalette::Window, Animal_Color_Loght_msg);  // How to change the color of the box chosen
 //     ui->Animal_One->setPalette(Animal_Color_Loght_msg);
 //     emit Animal_Color_Loght_msg;
      spot = 2;
    }
    // Spot is the box number that the information below will be displayed in
    else if (spot == 2)
    {
      ui->Animal_Two->setText(Animal_Color_Light_msg);  // For the ui to display the text in Animal_Color_Light_msg
      ui->Animal_Two->setPalette(Animal_Color_Loght_msg);
      spot = 3;
    }
    // Spot is the box number that the information below will be displayed in
    else if (spot == 3)
    {
      ui->Animal_Three->setText(Animal_Color_Light_msg);  // For the ui to display the text in Animal_Color_Light_msg
      ui->Animal_Three->setPalette(Animal_Color_Loght_msg);
      spot = 1;
    }
  }
  if (Animal_Color_Light_msg == "turtle")
  {
    Animal_Color_Light_msg = "turtle";
    Animal_Color_Loght_msg.setRgb(0, 255, 0);  // Green
    if (spot == 1)
    {
      ui->Animal_One->setText(Animal_Color_Light_msg);
      ui->Animal_One->setPalette(Animal_Color_Loght_msg);
      spot = 2;
    }
    else if (spot == 2)
    {
      ui->Animal_Two->setText(Animal_Color_Light_msg);
      ui->Animal_Two->setPalette(Animal_Color_Loght_msg);
      spot = 3;
    }
    else if (spot == 3)
    {
      ui->Animal_Three->setText(Animal_Color_Light_msg);
      ui->Animal_Three->setPalette(Animal_Color_Loght_msg);
      spot = 1;
    }
  }
  if (Animal_Color_Light_msg == "platypus")
  {
      Animal_Color_Light_msg = "platypus";
    Animal_Color_Loght_msg.setRgb(0, 0, 255);  // Blue
    if (spot == 1)
    {
      ui->Animal_One->setText(Animal_Color_Light_msg);
      ui->Animal_One->setPalette(Animal_Color_Loght_msg);
      spot = 2;
    }
    else if (spot == 2)
    {
      ui->Animal_Two->setText(Animal_Color_Light_msg);
      ui->Animal_Two->setPalette(Animal_Color_Loght_msg);
      spot = 3;
    }
    else if (spot == 3)
    {
      ui->Animal_Three->setText(Animal_Color_Light_msg);
      ui->Animal_Three->setPalette(Animal_Color_Loght_msg);
      spot = 1;
    }
  }

}

// Not fully functional since it has a input not a output component
// For the heartbeat inputs uncomment the below code and add/edit the other sections above

//void HelloGui::Tech_Sever_Ip(const std_msgs::String::ConstPtr &server){
//  auto Tech_Sever_Ip_msg = QString::fromStdString(server->data.c_str());
//  ui->Techinal_Server_IP_Input->text(Tech_Server_Ip_Input);
//  ui->Techinal_Server_IP_Input->text();
//    ipp = ui->Techinal_Server_IP_Input->text();
//  auto Techinal_Server_IP_msg = ui->Techinal_Server_IP_Input->text().toStdString();
//  auto Techinal_Server_IP_msg = ui->Techinal_Server_IP_Input->text();
//  QString Tech_Sever_Ip_msg = ui->Techinal_Server_IP_Input->text();
//  Tech_Sever_Ip_msg.toStdString();
//  Server_IP_pub_.publish(Techinal_Server_IP_msg);
//}

//void HelloGui::on_Server_IP_Input_Box_released()
//{
//    ui->Techinal_Server_IP_Input->text();
//    connect(ui->Server_IP_Input_Box, &QPushButton::released, ui->Techinal_Server_IP_Input->text());
//    Server_IP_pub_.publish(ui->Techinal_Server_IP_Input->text());
//}

//void HelloGui::Team_INfo(const std_msgs::String::ConstPtr &info){
//  inf = ui->Techinal_Server_IP_Input->text();

//  if (inf != inf)
//  {
//    inf = ui->Team_Info_Input->text();
//    Team_INfo_pub_.publish(inf);

//  }
//}

//void HelloGui::PORt(const std_msgs::String::ConstPtr &por){
//  ipo = ui->Port_Input->text();

//  if (ipo != inf)
//  {
//    ipo = ui->Port_Input->text();
//    PORt_pub_.publish(ipo);

//  }
//}

//===================================== Below is the HeartBeat code ===============================================

/*
 * Method Name: heartbeat_sock constructor
 * Purpose: open socket at specified ip and port
 * Matthew Bowen <mbowen2@lssu.edu>
 * 1/17/22
 */
heartbeat_sock::heartbeat_sock(const char ip[MAX], const int port)
{
    struct sockaddr_in servaddr, cli;

  // socket create and verification
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1)
  {
    printf("socket creation failed...\n");
    return;
  }
  else
    printf("Socket successfully created..\n");
  bzero(&servaddr, sizeof(servaddr));

  // assign IP, PORT
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(ip);
  servaddr.sin_port = htons(port);

  // connect the client socket to server socket
  if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0)
  {
    printf("connection with the server failed...\n");
    return;
  }
  else
    printf("connected to the server..\n");
}

/*
 * Method Name: heartbeat_sock destructor
 * Purpose: Close socket
 * Matthew Bowen <mbowen2@lssu.edu>
 * 1/17/22
 */
heartbeat_sock::~heartbeat_sock()
{
    // close the socket
  close(sockfd);
  close(connfd);
}

/*
 * Method Name: send
 * Purpose: send a formatted string to the RobotX Judges Server to convey system status
 * Function: recieve buffer with all information to send, comma separated,
 *              calculate XOR checksum, prepend '$', append terminating '*' <XOR checksum> "<CR><LF>",
 *              and send to the specified socket
 * Matthew Bowen <mbowen2@lssu.edu>
 * 1/17/22
 */
int heartbeat_sock::send(const char *buff)
{
    char packet[MAX];
    int buffLen = (int)strnlen(buff,MAX);
    unsigned char checksum = 0;
    char formatstring[MAX];

    //calculate checksum, skipping '$'
    for(int i=0; i<buffLen; i++)
        checksum ^= buff[i];
    //prepend '$'
    strcpy(packet, "$");
    //put buffer info into the packet
    strncat(packet, buff, MAX);
    //append '*' terminator, checksum, and <CR><LF> terminator
    sprintf(formatstring, "*%02X\r\n", checksum);
    strncat(packet,formatstring, MAX);

  //TESTING
  //print entire packet
  //packet[sizeof(packet)] = '\0';
  //printf("PACKET: %s\n", packet);

    //write the packet to the socket
    write(sockfd, packet, strnlen(packet,MAX));
    return 0;
}

//Temporary Constants - Delete once subscriptions are successful
const int lat = 0;
const int lon = 0;
const char teamid[] = "LSSUS";
const int status = 0;
const int UAVstat = 0;
//Parameter for when heartbeat is a subscriber - const std_msgs::String::ConstPtr &msg
void HelloGui::heartbeat()
{
  char packet[MAX];
  char timeDateStr[14];
  char latLonStr[32];
  char temp[64];
  time_t rawtime;
  struct tm * timeinfo;

 heartbeat_sock sock(IP,PORT);

    //compile the message to send
    //heartbeat message
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    //switch(task)
    //{
      //case 1:
        //Task Identifier
        strcpy(packet, "RXHRB,");
        //Local Date and Time
        strftime(timeDateStr,8,"%d%m%y,%H%M%S,",timeinfo);
        strcat(packet,timeDateStr);
        //Latitude and Longitude
        sprintf(latLonStr, "%.5f,%c,%.5f,%c,", lat, lat>0 ? 'N' : 'S', lon, lon>0 ? 'E' : 'W');
        strcat(packet, latLonStr);
        //Team Identifier
        strcat(packet,teamid);
        //System Status
        sprintf(temp, "%i,", status);
        strcat(packet,temp);
        //UAV Status
        sprintf(temp, "%i", UAVstat);
   // };
  ROS_INFO("Heartbeat : Working");

    //send the heartbeat
    sock.send(packet);
}
