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
        // Subscriber for jackal sweet selection
        sweet_selection_jackal_sub_ = nh_->subscribe("/sweet_selection_jackal", 10,
            &ROSWindowWrapper::sweetSelectionJackalCallback, this);
        ROS_INFO("Subscriber for /sweet_selection_jackal initialized.");
        // Publisher for goalposeselection
        goalpose_pub_ = nh_->advertise<std_msgs::Int32>("/sp4/goalpose", 10, true);
        ROS_INFO("Publisher for /sp4/goalpose initialized.");
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

void ROSWindowWrapper::triggerSweetSelectionGui(int selection){
    // Thread-safe call into Qt GUI thread
    QMetaObject::invokeMethod(window_, [this, selection]() {
        switch(selection) {
            case 0: window_->onZiel1Button(); break;
            case 1: window_->onZiel2Button(); break;
            case 2: window_->onZiel3Button(); break;
            case 3: window_->onZiel4Button(); break;
            default: break;
        }
    }, Qt::QueuedConnection);
}

void ROSWindowWrapper::publishGoalPoseId(int goalpose_id)
{
    if (!ros_enabled_) return;

    std_msgs::Int32 msg;
    msg.data = goalpose_id;
    goalpose_pub_.publish(msg);

    ROS_INFO("Published /sp4/goalpose: %d", goalpose_id);
}

void ROSWindowWrapper::sweetSelectionCallback(
    const std_msgs::Int32::ConstPtr& msg)
{
    if (!ros_enabled_) return;

    if (msg->data < 0 || msg->data > 3) {
        ROS_WARN("Invalid sweet selection received: %d", msg->data);
        return;
    }

    // Standard-Ablageposition
    publishGoalPoseId(1);

    ROS_INFO("Sweet selection %d → goalpose_id=1", msg->data);
    triggerSweetSelectionGui(msg->data);
}

void ROSWindowWrapper::sweetSelectionJackalCallback(
    const std_msgs::Int32::ConstPtr& msg)
{
    if (!ros_enabled_) return;

    if (msg->data < 0 || msg->data > 3) {
        ROS_WARN("Invalid jackal sweet selection received: %d", msg->data);
        return;
    }

    // Jackal-Ablageposition
    publishGoalPoseId(2);

    ROS_INFO("Jackal sweet selection %d → goalpose_id=2", msg->data);
    triggerSweetSelectionGui(msg->data);
}

void ROSWindowWrapper::publishStateChange(const std::string& state)
{
    if (!ros_enabled_) return;

    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);

    ROS_INFO_STREAM("Published state: " << state);

    // Reset nach Übergabe
    if (state == "OBJECT_GIVEN") {
        publishGoalPoseId(1);
        ROS_INFO("Reset goalpose to 1 after OBJECT_GIVEN");
    }
}

void ROSWindowWrapper::enableROS(bool enable) {
    ros_enabled_ = enable;
}

} // namespace pds
