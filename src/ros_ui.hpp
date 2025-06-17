#ifndef ROS_UI_H
#define ROS_UI_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <string>

#ifndef Q_MOC_RUN
#include "koordinator.hpp"
#include "dialogzustand.h"
#include "wartend.h"
#include "suchend.h"
#include "ausfuehrend.h"
#include "verabschiedend.h"
#endif
#include "Beobachter.h"
#include "udpnode.hpp"

namespace rosapi {

class ROSUI : public Beobachter {
public:
    ROSUI(int argc, char **argv, Koordinator* dlg);
    ~ROSUI();

    virtual void aktualisiere(Subjekt* s) override;

private:
    void sweetSelectionCallback(const std_msgs::Int32::ConstPtr& msg);
    void handleWartend(Wartend* dz);
    void handleSuchend(Suchend* dz);
    void handleAusfuehrend(Ausfuehrend* dz);
    void handleVerabschiedend(Verabschiedend* dz);
    void initialize();
    void updateState();
    void handleBildanalysatorMessage(const std::string& msg);

    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber sweet_sub_;
    ros::Timer state_timer_;
    Koordinator* dialog_;
    UDPNode* udp_node_;
    static const char* sweet_names_[4];
    int selected_sweet_;
    std::string last_state_;
};

} // namespace rosapi

#endif // ROS_UI_H
