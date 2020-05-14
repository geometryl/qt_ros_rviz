/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/main_window.h"
#include <QApplication>
#include <QtGui>

#include "ros/ros.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rviz_demo", ros::init_options::AnonymousName);
    //    std::map<std::string, std::string> robot_description;
    //    robot_description["textfile"] = "$(find excavator_description)/urdf/xiagong/xg_excavator_lidar.urdf";
    //    ros::param::set("robot_description", robot_description);

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    rviz_demo::MainWindow w(argc, argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();
}
