#include <ros/ros.h>
#include "global_planner.h"

using namespace hybrid_astar_search;

int main(int argc, char** argv){
  ros::init(argc, argv, "hybrid_astar_search");

  ros::NodeHandle nh("~");

  Global_Planner hybrid_astar_search;
  hybrid_astar_search.init(nh);

  ros::spin();

  return 0;
}
