#ifndef rviz_demo_MAIN_WINDOW_H
#define rviz_demo_MAIN_WINDOW_H

#include <ros/ros.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/display.h>
#include <rviz/panel.h>
#include <rviz/render_panel.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <QMainWindow>
#include "ui_main_window.h"

namespace rviz_demo {
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();

 public Q_SLOTS:
  void on_btn_select_clicked(bool check);
  void on_btn_movecamera_clicked(bool check);
  void on_btn_marker_clicked(bool check);
  void on_test_clicked(bool check);
  void on_test2_clicked(bool check);

 private:
  Ui::MainWindowDesign ui;

  rviz::RenderPanel *_render_panel = Q_NULLPTR;
  rviz::VisualizationManager *_manager = Q_NULLPTR;
  rviz::ToolManager *_tool_manager = Q_NULLPTR;
  rviz::Tool *_current_tool = Q_NULLPTR;
  rviz::Display *_urdf = Q_NULLPTR;
  rviz::Display *_pcd_livox = Q_NULLPTR;
  rviz::Display *_tf = Q_NULLPTR;
  rviz::Display *_marker = Q_NULLPTR;
};
}  // namespace rviz_demo

#endif  // rviz_demo_MAIN_WINDOW_H
