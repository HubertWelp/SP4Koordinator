#ifndef PDS_ROSWINDOWWRAPPER_H
#define PDS_ROSWINDOWWRAPPER_H

// Qt includes
#include <QObject>
#include <QString>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Project includes
#include "main_window.hpp"

namespace pds {

class MainWindow;

class ROSWindowWrapper : public QObject {
    Q_OBJECT
public:
    explicit ROSWindowWrapper(MainWindow* window, bool ros_enabled = true);
    ~ROSWindowWrapper();

    void enableROS(bool enable);

private Q_SLOTS:
    void onStateChanged(const QString& state);

private:
    void sweetSelectionCallback(const std_msgs::Int32::ConstPtr& msg);
    void publishStateChange(const std::string& state);
    void simulateButtonPress(int button_index);

    MainWindow* window_;
    bool ros_enabled_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sweet_selection_sub_;
    ros::Publisher state_pub_;
};

} // namespace pds

#endif // PDS_ROSWINDOWWRAPPER_H
