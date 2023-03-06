

/**
 * The #pragma once directive has a very simple concept. 
 * The header file containing this directive is included only once even if the programmer includes it multiple times during a compilation. 
 * This is not included in any ISO C++ standard. 
 * This directive works similar to the #include guard idiom. 
 * Use of #pragma once saves the program from multiple inclusion optimisation.
 * 
 */
#pragma once

// ROS includes
#include "ros/ros.h"

// Pose publishing
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// maps
#include "nav_msgs/OccupancyGrid.h"

// lama includes
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"
#include "lama/sdm/map.h"
#include "lama/pose2d.h"
#include "lama/image.h"

// Transform includes
#include <tf/tf.h>

class germgrid
{
private:

bool OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg); 
void computeIrradiatedRay(const Eigen::Vector3ui& from, const Eigen::Vector3ui& to, Eigen::VectorVector3ui& sink);   

private:
    // publishers
    ros::Publisher map_publisher;

    // maps
    lama::SimpleOccupancyMap *map1 = nullptr;
    nav_msgs::OccupancyGrid msg_map;

public:
    germgrid();
    ~germgrid();

    void publishMaps();
};




