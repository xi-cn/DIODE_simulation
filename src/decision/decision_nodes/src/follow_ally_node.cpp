#include "decision_nodes/follow_ally_node.h"

NodeStatus FollowAlly::onStart()
{
    //更新目标点
    if (!decision_base::Seek_Ally(robot_name, x, y)){
        return NodeStatus::SUCCESS;
    }

    decision_base::Set_Temp_Goal(x, y, false);
    return NodeStatus::RUNNING;
}

NodeStatus FollowAlly::onRunning()
{
    if (decision_base::Get_Trace_Robot_Pos(x, y, robot_name)){
        decision_base::Set_Temp_Goal(x, y, true);
        return NodeStatus::RUNNING;
    }
    else{
        return NodeStatus::SUCCESS;
    }
    
}

void FollowAlly::onHalted()
{
    ROS_INFO("stop tracing %s", robot_name.c_str());
}