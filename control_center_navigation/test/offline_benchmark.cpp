#define RELOCALIZATION_TEST
#ifdef RELOCALIZATION_BASELINE
#include "baseline_relocalization.cpp"
#else
#include "../src/cartographer_global_relocalization.cpp"
#endif
#include <fstream>
#include <iostream>
#include <cstdlib>
struct RelocalizationTest
{
  static void run(CartographerGlobalRelocalization & n)
  {
    const char * path=std::getenv("RELOCALIZATION_FIXTURE");
    if(!path) throw std::runtime_error("fixture missing");
    std::ifstream file(path);
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    double yaw;
    file>>map->info.width>>map->info.height>>map->info.resolution
      >>map->info.origin.position.x>>map->info.origin.position.y>>yaw;
    map->info.origin.orientation.w=std::cos(yaw/2);
    map->info.origin.orientation.z=std::sin(yaw/2);
    map->data.resize(map->info.width*map->info.height);
    for(auto & cell:map->data) {int value;file>>value;cell=static_cast<int8_t>(value);}
    size_t count;file>>count;std::vector<ScanPoint> points(count);
    for(auto & point:points) file>>point.x>>point.y;
    if(!file) throw std::runtime_error("invalid fixture");
    n.map_callback(map);
#ifndef RELOCALIZATION_BASELINE
    n.annotate_structure(points);
#endif
    Candidate best,second;const auto start=std::chrono::steady_clock::now();
    n.match(points,best,second);auto pose=n.candidate_to_pose(best);
    std::cout<<"BENCHMARK pose="<<pose[0]<<","<<pose[1]<<","<<pose[2]
      <<" score="<<best.score<<" second="<<second.score
      <<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
#ifndef RELOCALIZATION_BASELINE
    std::cout<<" evidence_valid="<<n.quality_valid(n.last_quality_);
#endif
    std::cout<<std::endl;
  }
};
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);int result=0;
  try {auto n=std::make_shared<CartographerGlobalRelocalization>();RelocalizationTest::run(*n);}
  catch(const std::exception & e) {std::cerr<<e.what()<<std::endl;result=1;}
  rclcpp::shutdown();return result;
}
