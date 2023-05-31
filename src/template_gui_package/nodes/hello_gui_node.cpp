#include <QApplication>
#include "hello_gui.h"
#include <stdlib.h> //

using namespace std;

int main(int argc, char *argv[])//
{               // For changing the new window made the hello_gui_node has to be changed in a lot of places
  ros::init(argc, argv, "hello_gui_node");
  QApplication a(argc, argv);

HelloGui w;
// set the window title as the node name
  w.setWindowTitle(QString::fromStdString(
                     ros::this_node::getName()));

  w.show();
  return a.exec();

}
