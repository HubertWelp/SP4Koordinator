#include "roswindowwrapper.h"

namespace pds {

ROSWindowWrapper::ROSWindowWrapper(MainWindow* window, bool ros_enabled)
    : window_(window)
    , ros_enabled_(ros_enabled) {
    if (ros_enabled_) {
        // Initialize ROS components
        nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

        // Subscribe to sweet selection topic
        sweet_selection_sub_ = nh_->subscribe("/sweet_selection", 10,
            &ROSWindowWrapper::sweetSelectionCallback, this);
        ROS_INFO("Subscriber for /sweet_selection initialized.");
        // Publisher for state changes
        state_pub_ = nh_->advertise<std_msgs::String>("/current_state", 10, true);
        ROS_INFO("Publisher for /current_state initialized.");

        // Connect to window's state change signal
        QObject::connect(window_, &MainWindow::stateChanged,
            [this](const QString& state) {
                if (ros_enabled_) {
                    this->publishStateChange(state.toStdString());
                }
            });
    }
}

ROSWindowWrapper::~ROSWindowWrapper() {
    if (ros_enabled_) {
        ROS_INFO("ROSWindowWrapper shutting down");
        // Cleanup happens automatically through unique_ptr and ROS shutdown
    }
}

void ROSWindowWrapper::onStateChanged(const QString& state) {
    if (!ros_enabled_) return;

    ROS_INFO_STREAM("State changed to: " << state.toStdString());
    this->publishStateChange(state.toStdString());
}

void ROSWindowWrapper::sweetSelectionCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (!ros_enabled_) return;

    if (msg->data >= 0 && msg->data < 4) {
        ROS_INFO("Received sweet selection: %d", msg->data);
        // Use QMetaObject::invokeMethod for thread-safe calls to GUI
        QMetaObject::invokeMethod(window_, [this, msg]() {
            switch(msg->data) {
                case 0: window_->onZiel1Button(); break;
                case 1: window_->onZiel2Button(); break;
                case 2: window_->onZiel3Button(); break;
                case 3: window_->onZiel4Button(); break;
            }
        }, Qt::QueuedConnection);
    } else {
        ROS_WARN("Invalid sweet selection received: %d", msg->data);
    }
}

void ROSWindowWrapper::publishStateChange(const std::string& state) {
    if (!ros_enabled_) return;

    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
    ROS_INFO_STREAM("Published state: " << state);
}

void ROSWindowWrapper::enableROS(bool enable) {
    ros_enabled_ = enable;
}

} // namespace pds
