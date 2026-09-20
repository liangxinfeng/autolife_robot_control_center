#undef NDEBUG
#include <cassert>
#include "../src/exact_goal_planner.cpp"
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("approach_planner_test");
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("approach_test_costmap");
  map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  map->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.29,0.29],[0.29,-0.29],[-0.29,-0.29],[-0.29,0.29]]")));
  assert(map->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS);
  auto grid=map->getCostmap();grid->resizeMap(120,120,.05,0.,0.);
  std::fill(grid->getCharMap(),grid->getCharMap()+14400,0);
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  node->declare_parameter("Approach.allow_occupied_approach",true);
  control_center_navigation::ExactGoalSmacPlanner approach,strict;
  approach.configure(node,"Approach",tf,map);strict.configure(node,"Strict",tf,map);
  geometry_msgs::msg::PoseStamped start,goal;
  start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=goal.pose.orientation.w=1.;
  start.pose.position.x=1.;start.pose.position.y=1.;goal.pose.position.x=4.;goal.pose.position.y=1.;
  grid->setCost(80,20,254);
  const std::vector<unsigned char> before(grid->getCharMap(),grid->getCharMap()+14400);
  auto path=approach.createPlan(start,goal,[]{return false;});assert(!path.poses.empty());
  auto waiting=path.poses.back();
  assert(std::hypot(waiting.pose.position.x-4.,waiting.pose.position.y-1.)>=.79);
  assert(before==std::vector<unsigned char>(grid->getCharMap(),grid->getCharMap()+14400));
  start.pose.position.x=2.;
  auto next=approach.createPlan(start,goal,[]{return false;});
  assert(next.poses.back().pose==waiting.pose); // stable endpoint across replans
  bool rejected=false;try{strict.createPlan(start,goal,[]{return false;});}
  catch(const nav2_core::GoalOccupied &){rejected=true;}assert(rejected);
  grid->setCost(80,20,0);
  auto exact=approach.createPlan(start,goal,[]{return false;});
  assert(exact.poses.back().pose==goal.pose); // clearing resumes the exact original target
  start.pose.position.x=3.6;grid->setCost(80,20,254);
  rejected=false;try{approach.createPlan(start,goal,[]{return false;});}
  catch(const nav2_core::GoalOccupied &){rejected=true;}assert(rejected); // do not reverse away when already nearby
  grid->setCost(80,20,254);std::fill(grid->getCharMap(),grid->getCharMap()+14400,254);
  rejected=false;try{approach.createPlan(start,goal,[]{return false;});}
  catch(const nav2_core::GoalOccupied &){rejected=true;}assert(rejected);
  approach.cleanup();strict.cleanup();map->on_cleanup(rclcpp_lifecycle::State());
  map.reset();node.reset();rclcpp::shutdown();
}
