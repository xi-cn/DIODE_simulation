#ifndef FOLLOW_ALLY_NODE
#define FOLLOW_ALLY_NODE

#include "decision_nodes/decision_base.h"
#include "behaviortree_cpp/behavior_tree.h"
#include <ros/ros.h>

using BT::NodeStatus;

class FollowAlly : public BT::StatefulActionNode
{
public:
    FollowAlly(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal") };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    double x, y;
    std::string robot_name;
};

#endif