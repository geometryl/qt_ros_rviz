#include "../include/rvizwidget.h"

#include <QDebug>
#include <QVBoxLayout>
#include <QVector3D>

#include <rviz/properties/parse_color.h>

RvizWidget::RvizWidget(QWidget *parent) : QWidget(parent), ui(new Ui::RvizWidget) {
    ui->setupUi(this);

    QVBoxLayout *layout = new QVBoxLayout();
    setLayout(layout);

    // rviz container
    render_panel_ = new rviz::RenderPanel();
    // mouse cursor
    //    render_panel_->setCursor(Qt::PointingHandCursor);
    layout->addWidget(render_panel_);

    // rviz control object
    manager_ = new rviz::VisualizationManager(render_panel_);
    // get tool manager object
    //    tool_manager_ = manager_->getToolManager();
    // rviz camera(translate rotate scale)
    render_panel_->initialize(manager_->getSceneManager(), manager_);

    // tool manager
    //    connect(tool_man, SIGNAL(toolAdded(Tool *)), this, SLOT(addTool(Tool *)));

    manager_->initialize();
    //    tool_manager_->initialize();
    manager_->removeAllDisplays();

    set_global_options();
    //    show_grid();
    //    show_map(true, "map_rviz", 10, "");
    //    show_pcd(true, "PointCloud");
    //    show_tf(true);
    show_urdf();
}

RvizWidget::~RvizWidget() {
    delete ui;
    ros::shutdown();
}

void RvizWidget::set_global_options() {
    manager_->setFixedFrame("base_link");
    // manager_->updateFixedFrame();
    manager_->setProperty("Background Color", QColor(0, 48, 48));
    //    manager_->updateBackgroundColor();
    render_panel_->setBackgroundColor(rviz::qtToOgre(QColor(0, 48, 48)));
    manager_->setProperty("Frame Rate", 30);
    //    manager_->updateFps();
    manager_->setProperty("Default Light", true);
    manager_->startUpdate();
}

void RvizWidget::show_grid() {
    if (Q_NULLPTR != grid_) {
        delete grid_;
    }
    grid_ = manager_->createDisplay("rviz/Grid", "Grid", true);
    ROS_ASSERT(grid_ != Q_NULLPTR);

    // configure the grid display the way we like it
    grid_->subProp("Alpha")->setValue(0.5);
    grid_->subProp("Cell Size")->setValue(1);
    grid_->subProp("Color")->setValue(QColor(100, 200, 250));
    grid_->setEnabled(true);
    grid_->subProp("Line Style")->setValue("Billboards"); // Lines
    grid_->subProp("Line Style")->subProp("Line Width")->setValue(0.03);
    grid_->subProp("Normal Cell Count")->setValue(0);
    grid_->subProp("Offset")->subProp("X")->setValue(0);
    grid_->subProp("Offset")->subProp("Y")->setValue(0);
    grid_->subProp("Offset")->subProp("Z")->setValue(0);
    grid_->subProp("Plane")->setValue("XY");
    grid_->subProp("Plane Cell Count")->setValue(10);
    grid_->subProp("Reference Frame")->setValue("Fixed Frame");
    manager_->startUpdate();
}

void RvizWidget::show_map(bool enable, QString topic, double alpha, QString color_scheme) {
    if (!enable && Q_NULLPTR != map_) {
        map_->setEnabled(enable);
        return;
    }

    if (Q_NULLPTR != map_) {
        ROS_ASSERT(map_);
        delete map_;
    }

    map_ = manager_->createDisplay("rviz/Map", "QMap", true);
    ROS_ASSERT(map_);
    map_->subProp("Topic")->setValue(topic);
    map_->subProp("Alpha")->setValue(alpha);
    map_->subProp("Color Scheme")->setValue(color_scheme);
    map_->setEnabled(enable);
    manager_->startUpdate();
}

void RvizWidget::show_pcd(bool enable, QString topic) {
    if (Q_NULLPTR != pcd_) {
        delete pcd_;
        pcd_ = manager_->createDisplay("rviz/PointCloud", "QLaser", enable);
        ROS_ASSERT(pcd_);
        pcd_->subProp("Topic")->setValue(topic);
    }
    pcd_->setEnabled(enable);
    manager_->startUpdate();
}

void RvizWidget::show_tf(bool enable) {
    if (Q_NULLPTR != tf_) {
        delete tf_;
    }
    tf_ = manager_->createDisplay("rviz/TF", "TF", enable);
}

void RvizWidget::show_urdf() {
    if (Q_NULLPTR != urdf_) {
        delete urdf_;
    }
    urdf_ = manager_->createDisplay("rviz/RobotModel", "Urdf", true);
    urdf_->subProp("Alpha")->setValue(1);
    urdf_->subProp("Collision Enabled")->setValue(false);
    urdf_->setEnabled(true);
    urdf_->subProp("Links")->subProp("All Links Enabled")->setValue(true);
    urdf_->subProp("Links")->subProp("Expand Joint Details")->setValue(false);
    urdf_->subProp("Links")->subProp("Expand Link Details")->setValue(false);
    urdf_->subProp("Links")->subProp("Expand Link Details")->setValue(false);
    urdf_->subProp("Links")->subProp("Expand Tree")->setValue(false);
    urdf_->subProp("Links")->subProp("Link Tree Style")->setValue("Links in Alphabetic Order");
    urdf_->subProp("Links")->subProp("base_link")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("base_link")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("base_link")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("base_link")->setValue(true);
    urdf_->subProp("Links")->subProp("end_effector")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("end_effector")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("end_effector")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_base")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("front_lidar_base")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_base")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_base")->setValue(true);
    urdf_->subProp("Links")->subProp("front_lidar_optical_frame")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("front_lidar_optical_frame")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_optical_frame")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_sp_optical_frame")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("front_lidar_sp_optical_frame")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("front_lidar_sp_optical_frame")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("hesai40")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("hesai40")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("hesai40")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("link1")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("link1")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("link1")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("link1")->setValue(true);
    urdf_->subProp("Links")->subProp("link2")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("link2")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("link2")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("link2")->setValue(true);
    urdf_->subProp("Links")->subProp("link3")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("link3")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("link3")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("link3")->setValue(true);
    urdf_->subProp("Links")->subProp("link4")->subProp("Alpha")->setValue(1);
    urdf_->subProp("Links")->subProp("link4")->subProp("Show Axes")->setValue(false);
    urdf_->subProp("Links")->subProp("link4")->subProp("Show Trail")->setValue(false);
    urdf_->subProp("Links")->subProp("link4")->setValue(true);
    urdf_->subProp("Robot Description")->setValue("robot_description");
    urdf_->subProp("TF Prefix")->setValue("");
    urdf_->subProp("Update Interval")->setValue(0);
    urdf_->setValue(true);
    urdf_->subProp("Visual Enabled")->setValue(true);

    manager_->startUpdate();
}
