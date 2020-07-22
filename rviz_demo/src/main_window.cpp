#include "../include/main_window.h"
#include <rviz/display_group.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/parse_color.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/forwards.h>
#include <rviz/selection/selection_manager.h>
#include <QDebug>
#include <QVariant>
#include <QtGui>

namespace rviz_demo {
using namespace Qt;

MainWindow::MainWindow(QWidget *parent) {
  ui.setupUi(this);
  setWindowIcon(QIcon(":/images/icon.png"));

  _render_panel = new rviz::RenderPanel();
  ui.viz_layout->addWidget(_render_panel);
  _manager = new rviz::VisualizationManager(_render_panel);
  _tool_manager = _manager->getToolManager();
  _render_panel->initialize(_manager->getSceneManager(), _manager);
  _manager->initialize();
  _tool_manager->initialize();
  _manager->removeAllDisplays();

  // global options
  try {
    _manager->setFixedFrame("base_link");
    _manager->setProperty("Background Color", QColor(38, 41, 74));
    _render_panel->setBackgroundColor(rviz::qtToOgre(QColor(38, 41, 74)));
    _manager->setProperty("Frame Rate", 30);
    _manager->setProperty("Default Light", true);
  } catch (...) {
    std::cout << "set global options error.\n";
  }

  // urdf
  try {
    if (Q_NULLPTR != _urdf) {
      delete _urdf;
    }
    _urdf = _manager->createDisplay("rviz/RobotModel", "RobotModel", true);
    ROS_ASSERT(_urdf != Q_NULLPTR);
    _urdf->subProp("Alpha")->setValue(1.0);
    _urdf->subProp("Collision Enabled")->setValue(false);
    _urdf->setEnabled(true);

    // links
    _urdf->subProp("Links")->subProp("All Links Enabled")->setValue(true);
    _urdf->subProp("Links")->subProp("Expand Joint Details")->setValue(false);
    _urdf->subProp("Links")->subProp("Expand Link Details")->setValue(false);
    _urdf->subProp("Links")->subProp("Expand Tree")->setValue(false);
    _urdf->subProp("Links")
        ->subProp("Link Tree Style")
        ->setValue("Links in Alphabetic Order");

    // base link
    _urdf->subProp("Links")
        ->subProp("base_link")
        ->subProp("Alpha")
        ->setValue(1);
    _urdf->subProp("Links")
        ->subProp("base_link")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("base_link")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("base_link")->setValue(true);

    // end_effector
    _urdf->subProp("Links")
        ->subProp("end_effector")
        ->subProp("Alpha")
        ->setValue(1);
    _urdf->subProp("Links")
        ->subProp("end_effector")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("end_effector")
        ->subProp("Show Trail")
        ->setValue(false);

    // link1
    _urdf->subProp("Links")->subProp("link1")->subProp("Alpha")->setValue(1);
    _urdf->subProp("Links")
        ->subProp("link1")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("link1")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("link1")->setValue(true);

    // link2
    _urdf->subProp("Links")->subProp("link2")->subProp("Alpha")->setValue(1);
    _urdf->subProp("Links")
        ->subProp("link2")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("link2")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("link2")->setValue(true);

    // link3
    _urdf->subProp("Links")->subProp("link3")->subProp("Alpha")->setValue(1);
    _urdf->subProp("Links")
        ->subProp("link3")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("link3")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("link3")->setValue(true);

    // link4
    _urdf->subProp("Links")->subProp("link4")->subProp("Alpha")->setValue(1);
    _urdf->subProp("Links")
        ->subProp("link4")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("link4")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("link4")->setValue(true);

    // livox_frame
    _urdf->subProp("Links")
        ->subProp("livox_frame")
        ->subProp("Alpha")
        ->setValue(1);
    _urdf->subProp("Links")
        ->subProp("livox_frame")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("livox_frame")
        ->subProp("Show Trail")
        ->setValue(false);
    _urdf->subProp("Links")->subProp("livox_frame")->setValue(true);

    // map
    _urdf->subProp("Links")->subProp("map")->subProp("Alpha")->setValue(1);
    _urdf->subProp("Links")
        ->subProp("map")
        ->subProp("Show Axes")
        ->setValue(false);
    _urdf->subProp("Links")
        ->subProp("map")
        ->subProp("Show Trail")
        ->setValue(false);

    _urdf->subProp("Robot Description")->setValue("robot_description");
    _urdf->subProp("TF Prefix")->setValue("");
    _urdf->subProp("Update Interval")->setValue(0);
    _urdf->setValue(true);
    _urdf->subProp("Visual Enabled")->setValue(true);
  } catch (...) {
    std::cout << "show rviz robot model error.\n";
  }

  // pcd
  try {
    if (Q_NULLPTR != _pcd_livox) {
      delete _pcd_livox;
    }
    // livox pointcloud
    _pcd_livox =
        _manager->createDisplay("rviz/PointCloud2", "PointCloud2", true);
    ROS_ASSERT(_pcd_livox != Q_NULLPTR);
    _pcd_livox->subProp("Alpha")->setValue(1);
    _pcd_livox->subProp("Autocompute Intensity Bounds")->setValue(true);

    // Autocompute value bounds
    _pcd_livox->subProp("Autocompute Value Bounds")
        ->subProp("Max Value")
        ->setValue(5.70165682);
    _pcd_livox->subProp("Autocompute Value Bounds")
        ->subProp("Min Value")
        ->setValue(-1.64995718);
    _pcd_livox->subProp("Autocompute Value Bounds")->setValue(true);
    _pcd_livox->subProp("Axis")->setValue("Z");
    _pcd_livox->subProp("Channel Name")->setValue("intensity");
    _pcd_livox->subProp("Color")->setValue(QColor(255, 255, 255));
    _pcd_livox->subProp("Color Transformer")->setValue("AxisColor");
    _pcd_livox->subProp("Decay Time")->setValue(0);
    _pcd_livox->setEnabled(true);
    _pcd_livox->subProp("Invert Rainbow")->setValue(false);
    _pcd_livox->subProp("Max Color")->setValue(QColor(255, 255, 255));
    _pcd_livox->subProp("Max Intensity")->setValue(78);
    _pcd_livox->subProp("Min Color")->setValue(QColor(0, 0, 0));
    _pcd_livox->subProp("Min Intensity")->setValue(0);
    _pcd_livox->subProp("Position Transformer")->setValue("XYZ");
    _pcd_livox->subProp("Queue Size")->setValue(10);
    _pcd_livox->subProp("Selectable")->setValue(true);
    _pcd_livox->subProp("Size (Pixels)")->setValue(3);
    _pcd_livox->subProp("Size (m)")->setValue(0.01);
    _pcd_livox->subProp("Style")->setValue("Points");
    _pcd_livox->subProp("Topic")->setValue("/livox/lidar_all");
    _pcd_livox->subProp("Unreliable")->setValue(false);
    _pcd_livox->subProp("Use Fixed Frame")->setValue(true);
    _pcd_livox->subProp("Use rainbow")->setValue(true);
    _pcd_livox->setValue(true);
  } catch (...) {
    std::cout << "show rviz pcd error.\n";
  }

  // marker
  //  try {
  //    // marker
  //    if (Q_NULLPTR != _marker) {
  //      delete _marker;
  //    }
  //    _marker = manager_->createDisplay("rviz/Marker", "Marker", true);
  //    ROS_ASSERT(_marker != Q_NULLPTR);
  //    _marker->setEnabled(true);
  //    _marker->subProp("Marker Topic")->setValue("/marker_shape");
  //  } catch (...) {
  //    std::cout << "show rviz mark error.\n";
  //  }

  _manager->startUpdate();

  // current view
  rviz::ViewManager *view_manager = _manager->getViewManager();
  view_manager->setRenderPanel(_render_panel);
  view_manager->setCurrentViewControllerType("rviz/Orbit");
  view_manager->getCurrent()->subProp("Distance")->setValue(16.2965889);
  view_manager->getCurrent()
      ->subProp("Enable Stereo Rendering")
      ->subProp("Stereo Eye Separation")
      ->setValue(0.06);
  view_manager->getCurrent()
      ->subProp("Enable Stereo Rendering")
      ->subProp("Stereo Focal Distance")
      ->setValue(1);
  view_manager->getCurrent()
      ->subProp("Enable Stereo Rendering")
      ->subProp("Swap Stereo Eyes")
      ->setValue(false);
  view_manager->getCurrent()
      ->subProp("Enable Stereo Rendering")
      ->setValue(false);
  view_manager->getCurrent()
      ->subProp("Focal Point")
      ->subProp("X")
      ->setValue(2.27047825);
  view_manager->getCurrent()
      ->subProp("Focal Point")
      ->subProp("Y")
      ->setValue(0.400056332);
  view_manager->getCurrent()
      ->subProp("Focal Point")
      ->subProp("Z")
      ->setValue(1.13878322);
  view_manager->getCurrent()->subProp("Focal Shape Fixed Size")->setValue(true);
  view_manager->getCurrent()->subProp("Focal Shape Size")->setValue(0.05);
  view_manager->getCurrent()->subProp("Invert Z Axis")->setValue(false);
  view_manager->getCurrent()->subProp("Near Clip Distance")->setValue(0.01);
  view_manager->getCurrent()->subProp("Pitch")->setValue(0.479797);
  view_manager->getCurrent()->subProp("Target Frame")->setValue("Fixed Frame");
  view_manager->getCurrent()->setValue("Orbit (rviz)");
  view_manager->getCurrent()->subProp("Yaw")->setValue(1.84192);
}

MainWindow::~MainWindow() {}

void MainWindow::on_btn_select_clicked(bool check) {
  Q_UNUSED(check);

  _current_tool = _tool_manager->addTool("rviz/Select");
  //  rviz::Property *pro = _current_tool->getPropertyContainer();
  //  pro->subProp("Topic")->setValue("/select");
  _tool_manager->setCurrentTool(_current_tool);
  _manager->startUpdate();
}

void MainWindow::on_btn_movecamera_clicked(bool check) {
  Q_UNUSED(check);

  _current_tool = _tool_manager->addTool("rviz/MoveCamera");
  _tool_manager->setCurrentTool(_current_tool);
  _manager->startUpdate();
}

void MainWindow::on_btn_marker_clicked(bool check) {
  Q_UNUSED(check);

  rviz::PropertyTreeModel *model_ =
      _manager->getSelectionManager()->getPropertyModel();
  qDebug() << "row: " << model_->rowCount()
           << " column: " << model_->columnCount();
  for (int i = 0; i < model_->rowCount(); ++i) {
    QModelIndex idx = model_->index(i, 0);
    rviz::Property *pro_ = model_->getProp(idx);
    if ("Point" == pro_->getName().mid(0, 5)) {  // pointcloud
      rviz::VectorProperty *vec_pro_ = (rviz::VectorProperty *)pro_->childAt(0);
      Ogre::Vector3 vec = vec_pro_->getVector();
      qDebug() << "name: " << pro_->getName() << " vec: " << vec.x << " "
               << vec.y << " " << vec.z;
    } else if ("Link" == pro_->getName().mid(0, 4)) {  // link
      rviz::VectorProperty *vec_pro_ = (rviz::VectorProperty *)pro_->childAt(0);
      Ogre::Vector3 vec = vec_pro_->getVector();
      rviz::QuaternionProperty *qua_pro_ =
          (rviz::QuaternionProperty *)pro_->childAt(1);
      Ogre::Quaternion qua = qua_pro_->getQuaternion();
      qDebug() << "name: " << pro_->getName()
               << " child num: " << pro_->numChildren() << " vec: " << vec.x
               << " " << vec.y << " " << vec.z << " qua: " << qua.x << " "
               << qua.y << " " << qua.z << " " << qua.w;
    } else {
    }
  }
}

void MainWindow::on_test_clicked(bool check) {}

void MainWindow::on_test2_clicked(bool check) {}

}  // namespace rviz_demo
