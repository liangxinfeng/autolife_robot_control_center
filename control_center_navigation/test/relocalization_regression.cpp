// Actual production matcher, synthetic geometry, no motion or initialpose calls.
#define RELOCALIZATION_TEST
#include "../src/cartographer_global_relocalization.cpp"
#include <iostream>

struct RelocalizationTest
{
  static void check(bool condition, const char * message)
  {
    if (!condition) {throw std::runtime_error(message);}
    std::cout << "PASS " << message << std::endl;
  }
  static nav_msgs::msg::OccupancyGrid::SharedPtr room()
  {
    auto map=std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id="map"; map->info.resolution=0.05;
    map->info.width=140; map->info.height=120; map->info.origin.orientation.w=1;
    map->data.assign(140*120,0);
    for (int y=10;y<=110;++y) {map->data[y*140+10]=100;map->data[y*140+130]=100;}
    for (int x=10;x<=130;++x) {map->data[10*140+x]=100;map->data[110*140+x]=100;}
    // Unique alcove and two pillars break the empty rectangle's 180-degree symmetry.
    for(int y=10;y<=38;++y) {map->data[y*140+91]=100;}
    for(int y=60;y<69;++y) for(int x=40;x<47;++x) {map->data[y*140+x]=100;}
    for(int y=80;y<93;++y) for(int x=104;x<110;++x) {map->data[y*140+x]=100;}
    return map;
  }
  static std::vector<ScanPoint> scan(const nav_msgs::msg::OccupancyGrid & map,
    double x,double y,double yaw,bool chairs)
  {
    std::vector<ScanPoint> points;
    for (int ray=0;ray<360;++ray) {
      const double a=ray*2*kPi/360-kPi;
      for(double r=0.20;r<9;r+=0.015) {
        int ix=static_cast<int>((x+r*std::cos(a+yaw))/map.info.resolution);
        int iy=static_cast<int>((y+r*std::sin(a+yaw))/map.info.resolution);
        if(ix<0 || iy<0 || ix>=140 || iy>=120) break;
        if(map.data[iy*140+ix]>=65) {
          double measured=(chairs && ((ray>=15 && ray<45)||(ray>=170 && ray<220))) ?
            std::max(0.3,r-0.65) : r;
          points.push_back({measured*std::cos(a),measured*std::sin(a)});break;
        }
      }
    }
    return points;
  }
  static void run(CartographerGlobalRelocalization & n)
  {
    auto map=room(); n.map_callback(map);
    sensor_msgs::msg::LaserScan near_scan;
    near_scan.range_min=.04;near_scan.ranges.assign(7,.05);
    check(n.trusted_near_return(near_scan,3),"stable near returns inside specified sensor range are retained");
    near_scan.ranges[2]=.03;
    check(!n.trusted_near_return(near_scan,3),"below-minimum-range neighbour invalidates near evidence");
    near_scan.ranges[2]=.10;
    check(!n.trusted_near_return(near_scan,3),"near-range spike is not interpreted as a wall");
    near_scan.ranges[2]=.05;n.near_wall_matching_=false;
    check(!n.trusted_near_return(near_scan,3),"near-wall mode can be disabled independently");
    n.near_wall_matching_=true;
    auto points=scan(*map,3.21,2.34,0.19,false);
    for(auto & p:points) p.origin_valid=true;
    Candidate truth{3.21/0.05-0.5,2.34/0.05-0.5,0.19,0};
    auto offsets=n.rotate_scan(points,truth.yaw_in_map);
    auto q=n.evaluate_candidate(truth.cell_x,truth.cell_y,offsets);
    check(n.quality_valid(q),"asymmetric room has multidirectional stable evidence");
    check(n.visibility_quality(truth,points).conflict_ratio<0.03,
      "clean mapped room is not penalized as rays crossing walls");
    Candidate wall_probe{1.0/.05-.5,2.0/.05-.5,0.0,0.0};
    ScanPoint crossing{8.0,0.0};crossing.origin_valid=true;
    check(n.visibility_quality(wall_probe,{crossing}).conflict_ratio>0.9,
      "ray beyond a long mapped wall records a visibility contradiction");
    crossing.origin_x=5.75;
    check(n.visibility_quality(wall_probe,{crossing}).conflict_ratio<0.1,
      "visibility ray starts at its actual sensor origin, not robot centre");
    crossing.origin_valid=false;
    check(n.visibility_quality(wall_probe,{crossing}).rays==0,
      "missing sensor origins cannot authorize supplemental verification");
    auto clutter=scan(*map,3.21,2.34,0.19,true);
    auto cq=n.evaluate_candidate(truth.cell_x,truth.cell_y,n.rotate_scan(clutter,0.19));
    check(n.quality_valid(cq) && cq.score>=n.min_score_,"partial furniture occlusion keeps correct pose viable");
    check(n.evaluate_candidate(30,30,n.rotate_scan(clutter,1.5)).score<cq.score,
      "clutter does not outrank correct geometry at a wrong pose");
    std::vector<ScanPoint> wall;
    for(int i=0;i<100;++i) {wall.push_back({1.0,-1.5+i*0.03});}
    check(!n.quality_valid(n.evaluate_candidate(109.5,60,n.rotate_scan(wall,0))),
      "one wall alone cannot authorize localization");
    std::vector<ScanPoint> isolated(240,ScanPoint{0.2,0.2});
    check(!n.quality_valid(n.evaluate_candidate(39,59,n.rotate_scan(isolated,0))),
      "many duplicates at one pillar cannot authorize localization");
    Candidate seed=truth;seed.cell_x+=0.38;seed.cell_y-=0.26;
    seed.score=n.evaluate_candidate(seed.cell_x,seed.cell_y,offsets).score;
    auto refined=n.continuous_refine(seed,points);
    check(refined.score>=seed.score && std::hypot(refined.cell_x-truth.cell_x,
      refined.cell_y-truth.cell_y)<0.50,"continuous optimizer improves a sub-cell offset");
    for(bool chairs : {false,true}) {
      Candidate best,second;
      check(n.match(chairs?clutter:points,best,second),"full map search returns candidates");
      auto pose=n.candidate_to_pose(best);
      const double error=std::hypot(pose[0]-3.21,pose[1]-2.34);
      std::cout << "METRIC chairs="<<chairs<<" xy="<<error<<" yaw="
        <<angular_distance(pose[2],0.19)<<" score="<<best.score<<" second="<<second.score<<std::endl;
      check(error<0.10 && angular_distance(pose[2],0.19)<0.05,
        "full-map candidate remains near ground truth");
      check(std::abs(best.score-n.evaluate_candidate(best.cell_x,best.cell_y,
        n.rotate_scan(n.uniformly_sample(chairs?clutter:points,n.fine_scan_points_),best.yaw_in_map)).score)<1e-9,
        "final score is recomputed on fine points, never inherited from coarse");
    }
    // A nearby person/object is not automatically a mapped wall. Contradictory
    // close evidence must not authorize a remotely shifted pose.
    auto blocked=points;
    for(int i=0;i<24;++i) {
      ScanPoint p{-.265,(i-12)*.001};
      p.origin_x=-.215;p.origin_valid=true;p.near_return=true;
      blocked.push_back(p);
    }
    Candidate blocked_best,blocked_second;
    n.match(blocked,blocked_best,blocked_second);
    auto blocked_pose=n.candidate_to_pose(blocked_best);
    const bool blocked_accepted=n.near_mode_valid_ && n.quality_valid(n.last_quality_) &&
      blocked_best.score>=n.min_score_ &&
      (blocked_best.score-blocked_second.score>=n.min_score_margin_ || n.supplemental_unique_);
    check(!blocked_accepted || (std::hypot(blocked_pose[0]-3.21,blocked_pose[1]-2.34)<.10 &&
      angular_distance(blocked_pose[2],.19)<.05),
      "unmapped nearby obstruction cannot authorize a wrong-room pose");
    check(!n.close_points_.empty(),"nearby obstruction fixture exercises the close-evidence branch");
    auto near_wall=scan(*map,.79,2.34,0,false);
    for(auto & p:near_wall) p.origin_valid=true;
    for(int i=0;i<18;++i) {
      ScanPoint p{-.265,(i-9)*.001};
      p.origin_x=-.215;p.origin_valid=true;p.near_return=true;
      near_wall.push_back(p);
    }
    Candidate near_best,near_second;
    n.match(near_wall,near_best,near_second);
    auto near_pose=n.candidate_to_pose(near_best);
    check(n.close_points_.size()==18,"continuous near patch below twenty points is still used");
    check(std::hypot(near_pose[0]-.79,near_pose[1]-2.34)<.10 &&
      angular_distance(near_pose[2],0)<.05 && n.near_mode_valid_,
      "near-wall synthetic room stays aligned with known geometry");
    auto duplicate=room(); duplicate->info.width=280;duplicate->data.assign(280*120,0);
    for(int y=0;y<120;++y) for(int x=0;x<140;++x) {
      duplicate->data[y*280+x]=map->data[y*140+x];
      duplicate->data[y*280+x+140]=map->data[y*140+x];
    }
    n.map_callback(duplicate);
    Candidate ambiguous_best, ambiguous_second;
    check(n.match(points,ambiguous_best,ambiguous_second) && ambiguous_second.score>0 &&
      ambiguous_best.score-ambiguous_second.score<n.min_score_margin_,
      "identical rooms retain a remote runner-up and reject false uniqueness");
    check(!n.supplemental_unique_,"supplemental verification also rejects identical rooms");
    auto make_scan=[&](double age) {
      auto s=std::make_shared<sensor_msgs::msg::LaserScan>();
      s->header.stamp=n.now()-rclcpp::Duration::from_seconds(age); return s;
    };
    check(n.scan_wait_detail().find("前后雷达")!=std::string::npos,
      "empty caches explicitly report both missing lidars");
    n.scan_callback(make_scan(0.01),false);
    check(n.scan_wait_detail().find("后雷达无有效数据")!=std::string::npos,
      "missing rear lidar is not misreported as timestamp synchronization");
    n.front_scans_.clear();
    // No executor is running: verify caching works even with search disabled/busy.
    n.enabled_=false;n.matching_=true;
    auto front=make_scan(0.02), rear=make_scan(0.03);
    n.scan_callback(front,false);n.scan_callback(rear,true);
    sensor_msgs::msg::LaserScan::SharedPtr f,r;
    check(n.select_scan_pair(f,r),"cache is fresh even while disabled/matching");
    n.last_front_stamp_=rclcpp::Time(f->header.stamp).nanoseconds();
    n.last_rear_stamp_=rclcpp::Time(r->header.stamp).nanoseconds(); f.reset();r.reset();
    check(!n.select_scan_pair(f,r),"same frames cannot satisfy multiple confirmations");
    n.front_scans_.clear();n.rear_scans_.clear();n.last_front_stamp_=n.last_rear_stamp_=0;
    n.scan_pair_max_age_sec_=0.10;
    n.scan_callback(make_scan(0.01),false); n.scan_callback(make_scan(0.4),true);
    check(!n.select_scan_pair(f,r),"similar arrival times do not hide capture timestamp mismatch");
    check(n.scan_wait_detail().find("时间差过大")!=std::string::npos,
      "capture mismatch is separately diagnosed without loosening tolerance");
    n.front_scans_.clear();n.rear_scans_.clear();
    n.scan_callback(make_scan(2),false);n.scan_callback(make_scan(2),true);
    check(!n.select_scan_pair(f,r),"stale paired scans are rejected");
    check(n.scan_wait_detail().find("已过期")!=std::string::npos,
      "expired scans are reported separately");
    n.front_scans_.clear(); auto fresh=make_scan(0.01);
    n.scan_callback(fresh,false);n.scan_callback(fresh,false);n.scan_callback(make_scan(0.1),false);
    check(n.front_scans_.size()==1,"duplicate and old acquisition frames are ignored");
    // Full ROS-facing path, isolated test domain and synthetic static odometry.
    n.map_callback(map);n.front_scans_.clear();n.rear_scans_.clear();
    n.enabled_=true;n.completed_=false;n.last_front_stamp_=n.last_rear_stamp_=0;
    n.retry_interval_sec_=0.0;n.max_attempts_=0;n.required_consistent_matches_=3;
    geometry_msgs::msg::TransformStamped odom, laser;
    odom.header.frame_id="odom";odom.child_frame_id="base_link";odom.transform.rotation.w=1;
    n.tf_buffer_.setTransform(odom,"synthetic",true);
    laser.header.frame_id="base_link";laser.child_frame_id="test_laser";laser.transform.rotation.w=1;
    n.tf_buffer_.setTransform(laser,"synthetic",true);
    auto feed=[&](bool empty) {
      auto s=make_scan(0.01);s->header.frame_id="test_laser";
      s->angle_min=-kPi;s->angle_increment=2*kPi/360;s->range_min=.1;s->range_max=15;
      if(!empty) for(const auto & p:points) s->ranges.push_back(std::hypot(p.x,p.y));
      n.scan_callback(s,false);n.scan_callback(s,true);n.search_latest_pair();
    };
    feed(false);check(!n.completed_ && n.consistent_match_count_==1,"first good frame cannot publish success");
    feed(true);check(n.consistent_match_count_==0,"invalid scan breaks consecutive confirmation");
    feed(false);feed(false);check(!n.completed_,"two fresh confirmations still wait");
    feed(false);check(n.completed_,"three independent aligned stationary scans complete dry-run localization");
  }
};
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  int result=0;
  try {auto n=std::make_shared<CartographerGlobalRelocalization>();RelocalizationTest::run(*n);}
  catch(const std::exception & e) {std::cerr<<"FAIL "<<e.what()<<std::endl;result=1;}
  rclcpp::shutdown();return result;
}
