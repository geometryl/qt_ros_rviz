/**
 * @file /include/rviz_demo/main_window.hpp
 *
 * @brief Qt based gui for rviz_demo.
 *
 * @date November 2010
 **/
#ifndef rviz_demo_MAIN_WINDOW_H
#define rviz_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "qnode.h"
#include "rvizwidget.h"
#include "ui_main_window.h"
#include <QMainWindow>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rviz_demo {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings();  // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check);
    void on_button_open_clicked(bool check);
    void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    RvizWidget *viz_;
};

} // namespace rviz_demo

#endif // rviz_demo_MAIN_WINDOW_H
