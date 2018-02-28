/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * multi_lidar_calibrator.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include "multi_lidar_calibrator.h"

void RosMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
	//check we have an initialization from the user
	if(initialpose_quaternion_ == tf::Quaternion::getIdentity())
	{
		return;
	}

	pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);

	pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);

	pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	DownsampleCloud(in_parent_cloud, filtered_cloud, voxel_size_);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;

	ndt.setTransformationEpsilon(ndt_epsilon_);
	ndt.setStepSize(ndt_step_size_);
	ndt.setResolution(ndt_resolution_);

	ndt.setMaximumIterations(ndt_iterations_);

	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(in_parent_cloud);

	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

	Eigen::Quaternionf init_quaternion(initialpose_quaternion_.w(),
	                                  initialpose_quaternion_.x(),
	                                  initialpose_quaternion_.y(),
	                                  initialpose_quaternion_.z());

	Eigen::Matrix3f rotation_matrix = init_quaternion.toRotationMatrix();
	Eigen::Vector4f translation_vector(initialpose_position_.x(), initialpose_position_.y(), initialpose_position_.z(), 1.0);

	Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

	init_guess.block(0,0,3,3) = rotation_matrix;
	init_guess.rightCols<1>() = translation_vector;

	std::cout << init_guess << std::endl;

	ndt.align(*output_cloud, init_guess);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
	          << " score: " << ndt.getFitnessScore () << std::endl;

	std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;
	std::cout << ndt.getFinalTransformation() << std::endl;

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*in_parent_cloud, *output_cloud, ndt.getFinalTransformation());


	// timer end
	//auto end = std::chrono::system_clock::now();
	//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}

void RosMultiLidarCalibratorApp::InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose)
{
	tf::Quaternion pose_quaternion(in_initialpose->pose.pose.orientation.x,
	                 in_initialpose->pose.pose.orientation.y,
	                 in_initialpose->pose.pose.orientation.z,
	                 in_initialpose->pose.pose.orientation.w);

	//rotation
	initialpose_quaternion_ = pose_quaternion;

	//translation
	initialpose_position_.setX(in_initialpose->pose.pose.position.x);
	initialpose_position_.setY(in_initialpose->pose.pose.position.y);
	initialpose_position_.setZ(in_initialpose->pose.pose.position.z);
}

void RosMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
}

void RosMultiLidarCalibratorApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_parent_topic_str, points_child_topic_str;

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

	in_private_handle.param<double>("voxel_size", voxel_size_, 0.2);
	ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 200);
	ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

	//generate subscribers and synchronizer
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());
	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	ros::Subscriber initialpose_sub_ = node_handle_.subscribe("initialpose", 1,
	                                                          &RosMultiLidarCalibratorApp::InitialPoseCallback, this);

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&RosMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

}


void RosMultiLidarCalibratorApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeRosIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

RosMultiLidarCalibratorApp::RosMultiLidarCalibratorApp()
{
	initialpose_quaternion_ = tf::Quaternion::getIdentity();
}