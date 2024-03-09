#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/package.h>

using BT::NodeStatus;

int flag;
int num = 0;

class MoveNode : public BT::StatefulActionNode
{
public:
    MoveNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        n = num;
        num++;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("sec") };
    }

    NodeStatus onStart() override
    {
        int sec = 0;
        auto msg = getInput<int>("sec", sec);
        if (!sec)
        {
            sec = 5;
        }
        ROS_INFO("start to stay put for %d seconds", sec);
        //计算deadline
        using namespace std::chrono;
        deadline_ = system_clock::now() + milliseconds(sec*1000);
        return NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        ROS_INFO("the %d is running", n);
        times++;
        if (times == 50)
            flag = 1;
        if (std::chrono::system_clock::now() >= deadline_)
        {
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::RUNNING;
        }
    }
    void onHalted() override
    {
        ROS_INFO("timeout");
    }
private:
    std::chrono::system_clock::time_point deadline_;
    int n;
    int times = 0;
};

NodeStatus Condition()
{
    if (flag == 0){
        return NodeStatus::FAILURE;
    }
    else{
        return NodeStatus::SUCCESS;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bt_test");

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<MoveNode>("MoveNode");
    factory.registerSimpleCondition("Doing", std::bind(Condition));

    std::string path = ros::package::getPath("bt_test");
    factory.registerBehaviorTreeFromFile(path + "/tree/main_tree.xml");
    auto tree = factory.createTree("MainTree");
    tree.tickWhileRunning();

    return 0;
}
