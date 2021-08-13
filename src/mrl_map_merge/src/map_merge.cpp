/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <thread>

#include <map_merge/map_merge.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <cv.h>

namespace map_merge
{
MapMerge::MapMerge() : subscriptions_size_(0)
{
    ros::NodeHandle private_nh("~");
    std::string frame_id;
    std::string merged_map_topic;
    std::string save_map_topic;

    private_nh.param("merging_rate", merging_rate_, 4.0);
    private_nh.param("discovery_rate", discovery_rate_, 0.05);
    private_nh.param("estimation_rate", estimation_rate_, 0.5);
    private_nh.param("known_init_poses", have_initial_poses_, true);
    private_nh.param("estimation_confidence", confidence_threshold_, 1.0);
    private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
    private_nh.param<std::string>("robot_map_updates_topic",
                                  robot_map_updates_topic_, "map_updates");
    private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
    private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
    private_nh.param<std::string>("save_map_topic", save_map_topic, "save_map");

    private_nh.param<std::string>("map_odom",robot_map_odom_topic,"odom");
    private_nh.param<std::string>("robot_footprint",robot_footprint_topic_,"move_base/local_costmap/footprint");
    private_nh.param<std::string>("world_frame", world_frame_, "world");

    /* publishing */
    merged_map_publisher_ =
            node_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);

    save_map_subscriber_ =  node_.subscribe(save_map_topic, 50,& MapMerge::saveMapCallback, this);
}

void MapMerge::saveMapCallback(const std_msgs::String::ConstPtr& msg)
{
    mapMerging();
}

/*
 * Subcribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
    ROS_DEBUG("Robot discovery started.");

    ros::master::V_TopicInfo topic_infos;
    geometry_msgs::Transform init_pose;
    std::string robot_name;
    std::string map_topic;
    std::string map_odom_topic;
    std::string map_footprint_topic;
    std::string map_footprint_topic_publisher;
    std::string robot_path_topic_publisher;
    std::string map_updates_topic;
    std::string global_marker_topic_publisher;
    std::string victim_marker_topic;
    std::string marked_victims_publisher;

    ros::master::getTopics(topic_infos);
    // default msg constructor does no properly initialize quaternion
    init_pose.rotation.w = 1;  // create identity quaternion

    for (const auto& topic : topic_infos) {
        // we check only map topic
        if (!isRobotMapTopic(topic)) {
            continue;
        }

        robot_name = robotNameFromTopic(topic.name);
        if (robots_.count(robot_name)) {
            // we already know this robot
            continue;
        }

        if (have_initial_poses_ && !getInitPose(robot_name, init_pose)) {
            ROS_WARN("Couldn't get initial position for robot [%s]\n"
                     "did you defined parameters map_merge/init_pose_[xyz]? in robot "
                     "namespace? If you want to run merging without known initial "
                     "positions of robots please set `known_init_poses` parameter "
                     "to false. See relavant documentation for details.",
                     robot_name.c_str());
            continue;
        }

        ROS_INFO("adding robot [%s] to system", robot_name.c_str());
        {
            std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
            subscriptions_.emplace_front();
            ++subscriptions_size_;
        }

        // no locking here. robots_ are used only in this procedure
        MapSubscription& subscription = subscriptions_.front();
        robots_.insert({robot_name, &subscription});
        subscription.initial_pose = init_pose;

        /* subscribe callbacks */
        map_topic = ros::names::append(robot_name, robot_map_topic_);
        map_updates_topic =
                ros::names::append(robot_name, robot_map_updates_topic_);
        ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
        subscription.map_sub = node_.subscribe<nav_msgs::OccupancyGrid>(
                    map_topic, 50,
                    [this, &subscription](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            fullMapUpdate(msg, subscription);
        });
        ROS_INFO("Subscribing to MAP updates topic: %s.",
                 map_updates_topic.c_str());
        subscription.map_updates_sub =
                node_.subscribe<map_msgs::OccupancyGridUpdate>(
                    map_updates_topic, 50,
                    [this, &subscription](
                    const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
            partialMapUpdate(msg, subscription);
        });

        map_odom_topic = ros::names::append(robot_name, robot_map_odom_topic);
        ROS_INFO("Subscribing to odom topic: %s.", map_odom_topic.c_str());
        subscription.map_odom = node_.subscribe<nav_msgs::Odometry>(
                    map_odom_topic, 50,
                    [this, &subscription](const nav_msgs::Odometry::ConstPtr& msg) {
            odomUpdate(msg, subscription);
        });


        map_footprint_topic = ros::names::append(robot_name, robot_footprint_topic_);
        map_footprint_topic_publisher = ros::names::append(robot_name, "global_footprint");
        robot_path_topic_publisher = ros::names::append(robot_name, "global_path");
        global_marker_topic_publisher = ros::names::append(robot_name, "global_marker");
        victim_marker_topic = ros::names::append(robot_name, "victim_marker");
        marked_victims_publisher = ros::names::append(robot_name, "marked_victims");

        ROS_INFO("Subscribing to footprint topic: %s.", map_footprint_topic.c_str());
        subscription.map_footprint = node_.subscribe<geometry_msgs::PolygonStamped>(
                    map_footprint_topic, 50,
                    [this, &subscription](const geometry_msgs::PolygonStamped::ConstPtr& msg) {
            footprintUpdate(msg, subscription);
        });

        ROS_INFO("Subscribing to vitim marker topic: %s.", victim_marker_topic.c_str());
        subscription.victim_marker = node_.subscribe<std_msgs::Int32>(
                    victim_marker_topic, 50,
                    [this, &subscription](const std_msgs::Int32::ConstPtr& msg) {
            victimUpdate(msg, subscription);
        });


        subscription.global_footprint_publisher_  =
                node_.advertise<geometry_msgs::PolygonStamped>(map_footprint_topic_publisher, 50, true);

        subscription.global_path_publisher_  =
                node_.advertise<nav_msgs::Path>(robot_path_topic_publisher, 50, true);

        subscription.global_marker_publisher_  =
                node_.advertise<visualization_msgs::MarkerArray>(global_marker_topic_publisher, 50, true);

        subscription.marked_victims_publisher_  =
                node_.advertise<visualization_msgs::MarkerArray>(marked_victims_publisher, 50, true);

        subscription.robot_name = robot_name;

        visualization_msgs::Marker marker3;

        marker3.header.frame_id = world_frame_;
        marker3.ns = subscription.robot_name+":path";
        marker3.id = 2;
        marker3.type = visualization_msgs::Marker::LINE_STRIP;

        marker3.action = visualization_msgs::Marker::ADD;

        marker3.scale.x = 0.1;
        marker3.scale.y = 0.1;
        marker3.scale.z = 0.1;
        marker3.color.a = 1.0; // Don't forget to set the alpha!
        marker3.color.r = 0.0;
        marker3.color.g = 1.0;
        marker3.color.b = 0.0;
        marker3.text = subscription.robot_name;
        subscription.global_path_marker = marker3;

    }
}

/*
 * mapMerging()
 */
void MapMerge::mapMerging()
{
    ROS_DEBUG("Map merging started.");

    if (have_initial_poses_) {
        std::vector<nav_msgs::OccupancyGridConstPtr> grids;
        std::vector<geometry_msgs::Transform> transforms;
        grids.reserve(subscriptions_size_);
        {
            boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
            for (auto& subscription : subscriptions_) {
                std::lock_guard<std::mutex> s_lock(subscription.mutex);
                grids.push_back(subscription.readonly_map);
                transforms.push_back(subscription.initial_pose);

            }
        }
        // we don't need to lock here, because when have_initial_poses_ is true we
        // will not run concurrently on the pipeline
        pipeline_.feed(grids.begin(), grids.end());
        pipeline_.setTransforms(transforms.begin(), transforms.end());
    }

    nav_msgs::OccupancyGridPtr merged_map;
    {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        merged_map = pipeline_.composeGrids();

    }
    if (!merged_map) {
        return;
    }

    ROS_DEBUG("all maps merged, publishing");
    ros::Time now = ros::Time::now();
    merged_map->info.map_load_time = now;
    merged_map->header.stamp = now;
    merged_map->header.frame_id = world_frame_;
    last_merged_map = merged_map;

    ROS_ASSERT(merged_map->info.resolution > 0.f);
    merged_map_publisher_.publish(merged_map);
}

void MapMerge::poseEstimation()
{
    ROS_DEBUG("Grid pose estimation started.");
    std::vector<nav_msgs::OccupancyGridConstPtr> grids;
    grids.reserve(subscriptions_size_);
    {
        boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
        for (auto& subscription : subscriptions_) {
            std::lock_guard<std::mutex> s_lock(subscription.mutex);
            grids.push_back(subscription.readonly_map);
        }
    }

    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    pipeline_.feed(grids.begin(), grids.end());
    // TODO allow user to change feature type
    pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
                                 confidence_threshold_);
}

void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                             MapSubscription& subscription)
{
    ROS_DEBUG("received full map update");
    std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map &&
            subscription.readonly_map->header.stamp > msg->header.stamp) {
        // we have been overrunned by faster update. our work was useless.
        return;
    }

    subscription.readonly_map = msg;
    subscription.writable_map = nullptr;
}


void MapMerge::odomUpdate(const nav_msgs::Odometry::ConstPtr& msg,
                          MapSubscription& subscription)
{
    std::lock_guard<std::mutex> lock(subscription.mutex);
    subscription.readonly_odom = msg;
    geometry_msgs::PoseStamped gPos;

    geometry_msgs::Transform init_pos =  subscription.initial_pose;

    gPos.pose.position.x = msg->pose.pose.position.x;
    gPos.pose.position.y = msg->pose.pose.position.y;
    gPos.pose.position.z = msg->pose.pose.position.z;
    gPos.pose.orientation.x =msg->pose.pose.orientation.x;
    gPos.pose.orientation.y =msg->pose.pose.orientation.y;
    gPos.pose.orientation.z =msg->pose.pose.orientation.z;
    gPos.pose.orientation.w =msg->pose.pose.orientation.w;

    geometry_msgs::PoseStamped lPos = subscription.last_global_pos;

    float dist = sqrt(pow((gPos.pose.position.x - lPos.pose.position.x),2)+pow((gPos.pose.position.y - lPos.pose.position.y),2)+pow((gPos.pose.position.z - lPos.pose.position.z),2));
    if(dist > 0.3)
    {
        subscription.last_global_pos = gPos;
        subscription.robot_path.poses.push_back(gPos);
        geometry_msgs::Point p;
        p.x = gPos.pose.position.x;
        p.y = gPos.pose.position.y;
        p.z = gPos.pose.position.z;
        subscription.global_path_marker.points.push_back(p);
    }

    ros::Time now = ros::Time::now();
    subscription.robot_path.header.stamp = now;
    subscription.robot_path.header.frame_id = world_frame_;

    subscription.global_path_publisher_.publish(subscription.robot_path);


    visualization_msgs::Marker marker1;
    marker1.header.frame_id = world_frame_;
    marker1.header.stamp = now;
    marker1.ns = subscription.robot_name+":arrow";
    marker1.id = 0;
    marker1.type = visualization_msgs::Marker::ARROW;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.position.x = gPos.pose.position.x;
    marker1.pose.position.y = gPos.pose.position.y;
    marker1.pose.position.z = gPos.pose.position.z;
    marker1.pose.orientation.x = gPos.pose.orientation.x;
    marker1.pose.orientation.y = gPos.pose.orientation.y;
    marker1.pose.orientation.z = gPos.pose.orientation.z;
    marker1.pose.orientation.w = gPos.pose.orientation.w;

    marker1.scale.x = 0.5;
    marker1.scale.y = 0.3;
    marker1.scale.z = 0.1;
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 0.0;
    marker1.color.g = 1.0;
    marker1.color.b = 0.0;
    marker1.text = subscription.robot_name;


    visualization_msgs::Marker marker2;
    marker2.header.frame_id = world_frame_;
    marker2.header.stamp = now;
    marker2.ns = subscription.robot_name+":title";
    marker2.id = 1;
    marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = gPos.pose.position.x+0.3;
    marker2.pose.position.y = gPos.pose.position.y+0.4;
    marker2.pose.position.z = gPos.pose.position.z;
    marker2.pose.orientation.x = gPos.pose.orientation.x;
    marker2.pose.orientation.y = gPos.pose.orientation.y;
    marker2.pose.orientation.z = gPos.pose.orientation.z;
    marker2.pose.orientation.w = gPos.pose.orientation.w;

    marker2.scale.x = 0.4;
    marker2.scale.y = 0.4;
    marker2.scale.z = 0.4;
    marker2.color.a = 1.0; // Don't forget to set the alpha!
    marker2.color.r = 0.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;

    marker2.text = subscription.robot_name;
    visualization_msgs::MarkerArray mArray;


    mArray.markers.push_back(marker1);
    mArray.markers.push_back(marker2);
    mArray.markers.push_back(subscription.global_path_marker);
    subscription.global_path_marker.header.stamp = now;
    subscription.global_marker_publisher_.publish(mArray);
}

void MapMerge::victimUpdate(const std_msgs::Int32::ConstPtr &msg, MapSubscription &subscription)
{
    ros::Time now = ros::Time::now();

    switch (msg->data) {
    case 1:
    {
        visualization_msgs::Marker marker1;
        marker1.header.frame_id = world_frame_;
        marker1.header.stamp = now;
        marker1.ns = subscription.robot_name+":alive_victim";
        marker1.id = subscription.marked_victims.markers.size();
        marker1.type = visualization_msgs::Marker::SPHERE;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = subscription.last_global_pos.pose.position.x;
        marker1.pose.position.y = subscription.last_global_pos.pose.position.y;
        marker1.pose.position.z = subscription.last_global_pos.pose.position.z;
        marker1.pose.orientation.x = subscription.last_global_pos.pose.orientation.x;
        marker1.pose.orientation.y = subscription.last_global_pos.pose.orientation.y;
        marker1.pose.orientation.z = subscription.last_global_pos.pose.orientation.z;
        marker1.pose.orientation.w = subscription.last_global_pos.pose.orientation.w;

        marker1.scale.x = 0.5;
        marker1.scale.y = 0.5;
        marker1.scale.z = 0.1;
        marker1.color.a = 1.0; // Don't forget to set the alpha!
        marker1.color.r = 0.0;
        marker1.color.g = 1.0;
        marker1.color.b = 0.0;
        marker1.text = subscription.robot_name+":alive_victim";
        subscription.marked_victims.markers.push_back(marker1);
        break;
    }
    case 2:
    {
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = world_frame_;
        marker2.header.stamp = now;
        marker2.ns = subscription.robot_name+":dead_victim";
        marker2.id = subscription.marked_victims.markers.size();
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.pose.position.x = subscription.last_global_pos.pose.position.x;
        marker2.pose.position.y = subscription.last_global_pos.pose.position.y;
        marker2.pose.position.z = subscription.last_global_pos.pose.position.z;
        marker2.pose.orientation.x = subscription.last_global_pos.pose.orientation.x;
        marker2.pose.orientation.y = subscription.last_global_pos.pose.orientation.y;
        marker2.pose.orientation.z = subscription.last_global_pos.pose.orientation.z;
        marker2.pose.orientation.w = subscription.last_global_pos.pose.orientation.w;

        marker2.scale.x = 0.5;
        marker2.scale.y = 0.5;
        marker2.scale.z = 0.1;
        marker2.color.a = 1.0; // Don't forget to set the alpha!
        marker2.color.r = 1.0;
        marker2.color.g = 0.0;
        marker2.color.b = 0.0;
        marker2.text = subscription.robot_name+":dead_victim";
        subscription.marked_victims.markers.push_back(marker2);
        break;
    }
    }
    subscription.marked_victims_publisher_.publish(subscription.marked_victims);

}

void MapMerge::footprintUpdate(const geometry_msgs::PolygonStamped::ConstPtr &msg, MapSubscription &subscription)
{

    std::lock_guard<std::mutex> lock(subscription.mutex);
    geometry_msgs::PolygonStamped mPoly;
    
    geometry_msgs::Transform init_pos =  subscription.initial_pose;
    for (geometry_msgs::Point32 p: msg->polygon.points) {
        geometry_msgs::Point32 new_point;
        new_point.x = p.x + init_pos.translation.x;
        new_point.y = p.y + init_pos.translation.y;
        new_point.z = p.z + init_pos.translation.z;
        mPoly.polygon.points.push_back(new_point);
    }

    ros::Time now = ros::Time::now();
    mPoly.header.stamp = now;
    mPoly.header.frame_id = world_frame_;

    subscription.global_footprint_publisher_.publish(mPoly);

}


void MapMerge::partialMapUpdate(
        const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
        MapSubscription& subscription)
{
    ROS_DEBUG("received partial map update");

    if (msg->x < 0 || msg->y < 0) {
        ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
                  msg->y);
        return;
    }

    size_t x0 = static_cast<size_t>(msg->x);
    size_t y0 = static_cast<size_t>(msg->y);
    size_t xn = msg->width + x0;
    size_t yn = msg->height + y0;

    nav_msgs::OccupancyGridPtr map;
    nav_msgs::OccupancyGridConstPtr readonly_map;  // local copy
    {
        // load maps
        std::lock_guard<std::mutex> lock(subscription.mutex);
        map = subscription.writable_map;
        readonly_map = subscription.readonly_map;
    }

    if (!readonly_map) {
        ROS_WARN("received partial map update, but don't have any full map to "
                 "update. skipping.");
        return;
    }

    // we don't have partial map to take update, we must copy readonly map and
    // update new writable map
    if (!map) {
        map.reset(new nav_msgs::OccupancyGrid(*readonly_map));
    }

    size_t grid_xn = map->info.width;
    size_t grid_yn = map->info.height;

    if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
        ROS_WARN("received update doesn't fully fit into existing map, "
                 "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                 "map is: [0, %lu], [0, %lu]",
                 x0, xn, y0, yn, grid_xn, grid_yn);
    }

    // update map with data
    size_t i = 0;
    for (size_t y = y0; y < yn && y < grid_yn; ++y) {
        for (size_t x = x0; x < xn && x < grid_xn; ++x) {
            size_t idx = y * grid_xn + x;  // index to grid for this specified cell
            map->data[idx] = msg->data[i];
            ++i;
        }
    }
    // update time stamp
    map->header.stamp = msg->header.stamp;

    {
        // store back updated map
        std::lock_guard<std::mutex> lock(subscription.mutex);
        if (subscription.readonly_map &&
                subscription.readonly_map->header.stamp > map->header.stamp) {
            // we have been overrunned by faster update. our work was useless.
            return;
        }
        subscription.writable_map = map;
        subscription.readonly_map = map;
    }
}

std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
    return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
    /* test whether topic is robot_map_topic_ */
    std::string topic_namespace = ros::names::parentNamespace(topic.name);
    bool is_map_topic =
            ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

    /* test whether topic contains *anywhere* robot namespace */
    auto pos = topic.name.find(robot_namespace_);
    bool contains_robot_namespace = pos != std::string::npos;

    /* we support only occupancy grids as maps */
    bool is_occupancy_grid = topic.datatype == "nav_msgs/OccupancyGrid";

    /* we don't want to subcribe on published merged map */
    bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

    return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
            is_map_topic;
}

/*
 * Get robot's initial position
 */
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::Transform& pose)
{
    std::string merging_namespace = ros::names::append(name, "map_merge");
    double yaw = 0.0;

    bool success =
            ros::param::get(ros::names::append(merging_namespace, "init_pose_x"),
                            pose.translation.x) &&
            ros::param::get(ros::names::append(merging_namespace, "init_pose_y"),
                            pose.translation.y) &&
            ros::param::get(ros::names::append(merging_namespace, "init_pose_z"),
                            pose.translation.z) &&
            ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"),
                            yaw);

    tf2::Quaternion q;
    q.setEuler(0., 0., yaw);
    pose.rotation = toMsg(q);

    return success;
}

/*
 * execute()
 */
void MapMerge::executemapMerging()
{
    ros::Rate r(merging_rate_);
    while (node_.ok()) {
        mapMerging();
        r.sleep();
    }
}

void MapMerge::executetopicSubscribing()
{
    ros::Rate r(discovery_rate_);
    while (node_.ok()) {
        topicSubscribing();
        r.sleep();
    }
}

void MapMerge::executeposeEstimation()
{
    if (have_initial_poses_)
        return;

    ros::Rate r(estimation_rate_);
    while (node_.ok()) {
        poseEstimation();
        r.sleep();
    }
}

/*
 * spin()
 */
void MapMerge::spin()
{
    ros::spinOnce();
    std::thread merging_thr([this]() { executemapMerging(); });
    std::thread subscribing_thr([this]() { executetopicSubscribing(); });
    std::thread estimation_thr([this]() { executeposeEstimation(); });
    ros::spin();
    estimation_thr.join();
    merging_thr.join();
    subscribing_thr.join();
}

}  // namespace map_merge

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_merge");
    // this package is still in development -- start wil debugging enabled
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    map_merge::MapMerge map_merging;
    map_merging.spin();
    return 0;
}
