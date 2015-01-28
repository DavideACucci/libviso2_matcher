/*
 * matcher_node,
 *
 * Copyright (C) 2015 Davide A. Cucci
 *
 * This file is part of libviso2_matcher.
 *
 * libviso2_matcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libviso2_matcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with libviso2_matcher.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * matcher_node.h
 *
 *  Created on: Jan 28, 2015
 *      Author: davide
 */

#ifndef EXTRACTOR_H_
#define EXTRACTOR_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "libviso2_matcher_wrapper.h"
#include "feature_manager.h"

class MatcherNode
{
public:
  MatcherNode(std::string imageTopic);

  virtual ~MatcherNode();

  virtual void handleImage(const sensor_msgs::ImageConstPtr& msg);

	bool getImage(const sensor_msgs::ImageConstPtr& msg,
				cv_bridge::CvImagePtr& cv_mono_ptr);

protected:
	void publishFeatures(const std::string& frame_id, ros::Time stamp);
	void display(cv_bridge::CvImagePtr cv_ptr);

private:
	void drawKeypoints(cv::Mat& frame);

private:

	// feature matcher
	libviso2MatcherWrapper *_matcher;
	bool _first;

	// id manader
	FeatureManager _fmanager;

	//Ros management
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSubscriber;

	//debug display
	std::string src_window;
	bool hideNew;
};

#endif /* EXTRACTOR_H_ */
