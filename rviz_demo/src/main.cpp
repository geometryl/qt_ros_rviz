#include <QApplication>
#include <QtGui>
#include <iostream>
#include "../include/main_window.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_demo", ros::init_options::AnonymousName);
  if (!ros::master::check()) {
    std::cout << "ros initial failed.";
    return -1;
  }
  QApplication app(argc, argv);
  rviz_demo::MainWindow w;
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}
