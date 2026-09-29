#undef NDEBUG
#include <cassert>
#include <chrono>
#include <thread>
#include "behaviortree_cpp/bt_factory.h"
static bool blocked=true, bad_path=false, reached=false;
static int plans=0;
class Compute : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts(){return {BT::InputPort<std::string>("goal"),
    BT::InputPort<std::string>("planner_id"),BT::OutputPort<std::string>("path"),
    BT::OutputPort<int>("error_code_id")};}
  BT::NodeStatus tick() override {++plans;setOutput("error_code_id",bad_path?206:0);
    setOutput("path",blocked?"safe_wait":"original");
    return bad_path?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;}
};
class Follow : public BT::SyncActionNode {
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts(){return {BT::InputPort<std::string>("path"),
    BT::InputPort<std::string>("controller_id"),BT::OutputPort<int>("error_code_id")};}
  BT::NodeStatus tick() override {setOutput("error_code_id",0);
    reached=getInput<std::string>("path").value()=="original";return BT::NodeStatus::SUCCESS;}
};
class Wait : public BT::StatefulActionNode {
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts(){return {BT::InputPort<double>("wait_duration")};}
  BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::SUCCESS;}
  void onHalted() override {}
};
int main(int argc,char **argv){
  assert(argc==2); BT::BehaviorTreeFactory f;
  for(auto lib:{"pipeline_sequence","recovery_node","rate_controller","round_robin_node"})
    f.registerFromPlugin(std::string("/opt/ros/jazzy/lib/libnav2_")+lib+"_bt_node.so");
  f.registerNodeType<Compute>("ComputePathToPose");f.registerNodeType<Follow>("FollowPath");
  f.registerNodeType<Wait>("Wait");
  f.registerSimpleAction("ControllerSelector",[](BT::TreeNode &n){n.setOutput("selected_controller","FollowPath");return BT::NodeStatus::SUCCESS;},
    {BT::OutputPort<std::string>("selected_controller"),BT::InputPort<std::string>("default_controller"),BT::InputPort<std::string>("topic_name")});
  f.registerSimpleCondition("GoalReached",[](BT::TreeNode &){return reached?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;},
    {BT::InputPort<std::string>("goal"),BT::InputPort<std::string>("global_frame"),BT::InputPort<std::string>("robot_base_frame")});
  for(auto id:{"WouldAPlannerRecoveryHelp","WouldAControllerRecoveryHelp"})
    f.registerSimpleCondition(id,[](BT::TreeNode &){return BT::NodeStatus::FAILURE;},{BT::InputPort<int>("error_code")});
  f.registerSimpleCondition("GoalUpdated",[](BT::TreeNode &){return BT::NodeStatus::FAILURE;});
  f.registerSimpleAction("ClearEntireCostmap",[](BT::TreeNode &){return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("service_name")});
  f.registerSimpleAction("Spin",[](BT::TreeNode &){return BT::NodeStatus::FAILURE;},{BT::InputPort<double>("spin_dist"),BT::OutputPort<int>("error_code_id")});
  f.registerSimpleAction("BackUp",[](BT::TreeNode &){return BT::NodeStatus::FAILURE;},{BT::InputPort<double>("backup_dist"),BT::InputPort<double>("backup_speed"),BT::OutputPort<int>("error_code_id")});
  auto bb=BT::Blackboard::create();bb->set("goal",std::string("original"));
  auto tree=f.createTreeFromFile(argv[1],bb);
  for(int i=0;i<130;++i){assert(tree.tickOnce()==BT::NodeStatus::RUNNING);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  assert(!reached && plans>=2); // reached staging is NEVER action success; periodic replan continues
  blocked=false;auto status=BT::NodeStatus::RUNNING;
  for(int i=0;i<150 && status==BT::NodeStatus::RUNNING;++i){status=tree.tickOnce();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  assert(status==BT::NodeStatus::SUCCESS && reached);
  tree.haltTree();blocked=true;bad_path=true;reached=false;
  assert(tree.tickOnce()==BT::NodeStatus::FAILURE); // preserve genuine planning failure, not infinite retry
  tree.haltTree();bad_path=false;
  assert(tree.tickOnce()==BT::NodeStatus::RUNNING);tree.haltTree();
  assert(tree.rootNode()->status()==BT::NodeStatus::IDLE);
}
