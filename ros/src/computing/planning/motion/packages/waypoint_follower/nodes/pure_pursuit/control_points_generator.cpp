/*
 *  Copyright (c) 2018, TierIV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "control_points_generator.h"

namespace waypoint_follower
{
ControlPointsGenerator::ControlPointsGenerator()
{
}

ControlPointsGenerator::~ControlPointsGenerator()
{
}

void ControlPointsGenerator::setConfig(const autoware_msgs::ConfigWaypointFollower& config)
{
  config_ = config;
}

void ControlPointsGenerator::setCurrentVelocity(const double& velocity)
{
  current_velocity_ = velocity;
}

autoware_msgs::lane ControlPointsGenerator::calcControlPoints(const autoware_msgs::lane& lane)
{
  autoware_msgs::lane control_points;
  geometry_msgs::Pose pose[2];
  for (int i = 0; i < 2; i++)
  {
    control_points.waypoints.push_back(lane.waypoints[i]);
  }
  pose[0] = lane.waypoints[1].pose.pose;
  for (int i = 1; i < lane.waypoints.size() - 1; i++)
  {
    pose[1] = lane.waypoints[i + 1].pose.pose;
    const double d_len2 = pow(getPlaneDistance(pose[1].position, pose[0].position), 2);
    if (d_len2 < 1e-8)
    {
      continue;
    }
    const double& vel = lane.waypoints[i + 1].twist.twist.linear.x;
    const geometry_msgs::Point rlt_local_target = calcRelativeCoordinate(pose[1].position, pose[0]);
    const double local_y_len = rlt_local_target.y;
    const int sgn = (local_y_len < 0.0) ? -1 : 1;
    const double radius = (fabs(local_y_len) < 1e-3) ? sgn * 1e4 : d_len2 / (2 * local_y_len);
    const double& min_len = config_.minimum_lookahead_distance;
    double lookahead = config_.lookahead_ratio * fabs(vel);
    lookahead = (lookahead < min_len) ? min_len : lookahead;
    const double l_len2 = lookahead * lookahead;
    geometry_msgs::Pose rlt_target;
    rlt_target.position.y = l_len2 / d_len2 * local_y_len;
    rlt_target.position.x = sqrt(l_len2 - rlt_target.position.y * rlt_target.position.y);
    const double yaw = asin(rlt_local_target.x / radius);
    rlt_target.orientation = tf::createQuaternionMsgFromYaw(yaw);
    autoware_msgs::waypoint wp(lane.waypoints[i]);
    wp.pose.pose = calcAbsoluteCoordinate(rlt_target, pose[0]);
    pose[0] = pose[1];
    if (std::isnan(wp.pose.pose.position.x) || std::isnan(wp.pose.pose.position.y))
    {
      continue;
    }
    control_points.waypoints.push_back(wp);
  }
  return control_points;
}
}
