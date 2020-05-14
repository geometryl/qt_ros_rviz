#ifndef RVIZWIDGET_H
#define RVIZWIDGET_H

#include "ui_rvizwidget.h"
#include <QWidget>

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/panel.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>

namespace Ui {
class RvizWidget;
}

class RvizWidget : public QWidget {
    Q_OBJECT

public:
    explicit RvizWidget(QWidget *parent = 0);
    ~RvizWidget();

    // global options
    void set_global_options(); // QString fixed_frame, QColor bg_color, int frame_rate
    // display grid
    void show_grid();

    // display map
    void show_map(bool enable, QString topic, double alpha, QString color_scheme);
    // display laser
    void show_pcd(bool enable, QString topic);
    // display navigate
    // display tf
    void show_tf(bool enable);
    // display urdf
    void show_urdf();

private:
    Ui::RvizWidget *ui;

    rviz::RenderPanel *render_panel_;
    rviz::VisualizationManager *manager_;

    rviz::Display *grid_;
    rviz::Display *map_;
    rviz::Display *pcd_;
    rviz::Display *urdf_;
    rviz::Display *tf_;

    rviz::Tool *current_tool;
    rviz::ToolManager *tool_manager_;
    QString node_name;
};

#endif // RVIZWIDGET_H
