#include "ros_ui.hpp"

namespace rosapi {

const char* ROSUI::sweet_names_[] = {"Maoam", "Snickers", "MilkyWay", "Schokoriegel"};

ROSUI::ROSUI(int argc, char **argv, Koordinator* dlg)
    : dialog_(dlg), selected_sweet_(-1), last_state_("") {
    ros::init(argc, argv, "sweet_selection_node");

    udp_node_ = new UDPNode(5843);
    udp_node_->connect(udp_node_, &UDPNode::msgReceivedSignal,
        [this](QString msg) {
            ROS_INFO("UDP Message received: %s", msg.toStdString().c_str());
            handleBildanalysatorMessage(msg.toStdString());
        });

    initialize();
    dialog_->anmelden(this);
    dialog_->getBildAnalysator()->anmelden(this);
    dialog_->getRoboter()->anmelden(this);

    std::cout << "Observer registration completed" << std::endl;

    state_timer_ = nh_.createTimer(ros::Duration(30.0),
        [this](const ros::TimerEvent&) {
            ROS_INFO("Timer triggered");
            dialog_->onTimer();
        });
    state_timer_.stop();
}

ROSUI::~ROSUI() {
    state_timer_.stop();
    if (dialog_) {
        dialog_->abmelden(this);
        dialog_->getBildAnalysator()->abmelden(this);
        dialog_->getRoboter()->abmelden(this);
    }
    delete udp_node_;
    ros::shutdown();
}

void ROSUI::initialize() {
    ros::NodeHandle nh;
    state_pub_ = nh.advertise<std_msgs::String>("/current_state", 10, true);
    sweet_sub_ = nh.subscribe("/sweet_selection", 10, &ROSUI::sweetSelectionCallback, this);
}

void ROSUI::handleBildanalysatorMessage(const std::string& msg) {
    std::cout << "ROSUI handling message: " << msg << std::endl;

    // Store current state for comparison
    auto current_state = dialog_->getDialogzustand();
    std::cout << "Current state type: " << typeid(*current_state).name() << std::endl;

    // Process the message
    dialog_->getBildAnalysator()->bildInfoAuswerten(msg);

    // Explicitly trigger state update since we might be missing observer notifications
    if (dynamic_cast<Suchend*>(current_state)) {
        std::cout << "In Suchend state, explicitly checking result" << std::endl;
        bool isError = dialog_->getBildAnalysator()->getNichtErkannt();
        if (isError) {
            std::cout << "Object not found, triggering nicht erkannt" << std::endl;
            dialog_->objektNichtErkannt();
        } else {
            std::cout << "Object found, triggering erkannt" << std::endl;
            dialog_->objektErkannt();
        }
        updateState();  // Force state update
    }
}

void ROSUI::sweetSelectionCallback(const std_msgs::Int32::ConstPtr& msg) {
    if(msg->data < 0 || msg->data > 3) {
        ROS_WARN("Invalid sweet ID %d", msg->data);
        return;
    }

    selected_sweet_ = msg->data;
    ROS_INFO("Processing selection: %s", sweet_names_[selected_sweet_]);

    state_timer_.stop();

    switch(selected_sweet_) {
        case 0:
            dialog_->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Maoam);
            break;
        case 1:
            dialog_->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Snickers);
            break;
        case 2:
            dialog_->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::MilkyWay);
            break;
        case 3:
            dialog_->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Schokoriegel);
            break;
    }
    ROS_INFO("Selection processed - transitioning to Suchend");
}


void ROSUI::updateState() {
    Dialogzustand* dz = dialog_->getDialogzustand();
    std::string current_state;

    if (Wartend* state = dynamic_cast<Wartend*>(dz)) {
        current_state = "Wartend";
        handleWartend(state);
        state_timer_.stop();  // Ensure timer is stopped
    }
    else if (Suchend* state = dynamic_cast<Suchend*>(dz)) {
        current_state = "Suchend";
        handleSuchend(state);
        state_timer_.stop();  // Ensure timer is stopped
    }
    else if (Ausfuehrend* state = dynamic_cast<Ausfuehrend*>(dz)) {
        current_state = "Ausfuehrend";
        handleAusfuehrend(state);
        // Reset and start timer for Ausfuehrend state
        state_timer_.stop();
        state_timer_.setPeriod(ros::Duration(30.0));
        state_timer_.start();
    }
    else if (Verabschiedend* state = dynamic_cast<Verabschiedend*>(dz)) {
        current_state = "Verabschiedend";
        handleVerabschiedend(state);
        // Reset timer for Verabschiedend state
        state_timer_.stop();
        state_timer_.setPeriod(ros::Duration(5.0));
        state_timer_.start();
    }

    if (last_state_ != current_state) {
        ROS_INFO("State transition: %s -> %s",
                 last_state_.empty() ? "NONE" : last_state_.c_str(),
                 current_state.c_str());
        last_state_ = current_state;

        // Publish state change
        std_msgs::String msg;
        msg.data = current_state;
        state_pub_.publish(msg);
    }
}

void ROSUI::aktualisiere(Subjekt* s) {
    if (s == dialog_) {
        std::cout << "\nDialog update received (COUT)\n";
        ROS_INFO("Dialog update received");
        updateState();
    }
    else if (dynamic_cast<Bildanalysator_Proxy*>(s)) {
        ROS_INFO("Bildanalysator update received");
        Bildanalysator_Proxy* ba = dynamic_cast<Bildanalysator_Proxy*>(s);

        float x = 0, y = 0, z = 0;
        ba->getObjektPosition(&x, &y, &z);
        float phi = ba->getObjektOrientierung();
        float breite = ba->getObjektBreite();
        ROS_INFO("Current values in update - x:%.2f y:%.2f z:%.2f phi:%.2f width:%.2f",
                 x, y, z, phi, breite);

        bool isError = ba->getNichtErkannt();
        ROS_INFO("Processing Bildanalysator result: %s", isError ? "Not Found" : "Found");

        if (isError) {
            ROS_INFO("Calling objektNichtErkannt");
            dialog_->objektNichtErkannt();
        } else {
            ROS_INFO("Calling objektErkannt");
            dialog_->objektErkannt();
        }
    }
    else if (dynamic_cast<Roboter*>(s)) {
        ROS_INFO("Robot update received");
        Roboter* robot = dynamic_cast<Roboter*>(s);
        std::string status = robot->getStatus();
        ROS_INFO("Robot status: %s", status.c_str());

        if (!status.empty()) {
            ROS_INFO("Processing robot completion");
            dialog_->objektUebergeben();
        }
    }
}

void ROSUI::handleWartend(Wartend* dz) {
    ROS_INFO("Handling Wartend state");
    std_msgs::String msg;
    msg.data = "WAITING_FOR_SELECTION";
    state_pub_.publish(msg);
    state_timer_.stop();
}

void ROSUI::handleSuchend(Suchend* dz) {
    ROS_INFO("Handling Suchend state");
    std_msgs::String msg;
    msg.data = "SEARCHING_FOR_" + std::string(sweet_names_[dz->getSuessigkeit()]);
    state_pub_.publish(msg);
    state_timer_.stop();
}

void ROSUI::handleAusfuehrend(Ausfuehrend* dz) {
    ROS_INFO("Handling Ausfuehrend state");
    std_msgs::String msg;
    msg.data = "EXECUTING_PICKUP";
    state_pub_.publish(msg);

    state_timer_.stop();
    state_timer_.setPeriod(ros::Duration(30.0));
    ROS_INFO("Starting execution timeout timer (30s)");
    state_timer_.start();
}

void ROSUI::handleVerabschiedend(Verabschiedend* dz) {
    ROS_INFO("Handling Verabschiedend state with trigger %d", dz->getAusloeser());
    std_msgs::String msg;

    switch(dz->getAusloeser()) {
        case 0:
            msg.data = "OBJECT_NOT_FOUND";
            break;
        case 1:
            msg.data = "REQUEST_TIMEOUT";
            break;
        case 2:
            msg.data = "OBJECT_GIVEN";
            break;
        default:
            msg.data = "REQUEST_TIMEOUT";
    }
    state_pub_.publish(msg);

    state_timer_.stop();
    state_timer_.setPeriod(ros::Duration(5.0));
    ROS_INFO("Starting transition timer (5s)");
    state_timer_.start();
}

} // namespace rosapi
