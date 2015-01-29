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
 * matcher_node.cpp
 *
 *  Created on: Jan 28, 2015
 *      Author: davide
 */

#include "matcher_node.h"

#include <opencv2/opencv.hpp>

#include <libviso2_matcher_msgs/FeatureArray.h>

using namespace cv;
using namespace std;

MatcherNode::MatcherNode(string imageTopic) :
    _nh("~"), _it(_nh), _first(true) {
  _imageSubscriber = _it.subscribe(imageTopic, 1, &MatcherNode::handleImage,
      this);

  libviso2MatcherWrapper::parameters params;

  _matcher = new libviso2MatcherWrapper(params);

  //debug window
  src_window = "Extracted Features";
  namedWindow(src_window, CV_WINDOW_AUTOSIZE);
  hideNew = true;

  _dcfg_f = boost::bind(&MatcherNode::dcfgCallback, this, _1, _2);
  _dcfg_server.setCallback(_dcfg_f);

  _featurePublisher = ros::NodeHandle("~").advertise<
      libviso2_matcher_msgs::FeatureArray>("features", 16);

}

void MatcherNode::handleImage(const sensor_msgs::ImageConstPtr& msg) {

  cv_bridge::CvImagePtr cv_mono_ptr;

  if (getImage(msg, cv_mono_ptr)) {
    _matcher->pushBack(cv_mono_ptr->image, false);

    if (!_first) {
      _matcher->matchFeatures(0, NULL);
      _fmanager.updateFeatures(_matcher->getMatches());

    }

    _first = false;

    display(cv_mono_ptr);

    publishFeatures("", msg->header.stamp);
  }
}

bool MatcherNode::getImage(const sensor_msgs::ImageConstPtr& msg,
    cv_bridge::CvImagePtr& cv_mono_ptr) {
  try {
    cv_mono_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    return true;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
}

void MatcherNode::publishFeatures(const string& frame_id, ros::Time stamp) {

  libviso2_matcher_msgs::FeatureArray msg;

  const FeatureManager::FeatureMap &features = _fmanager.getFeatures();

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.features.resize(features.size());

  int k = 0;
  for (auto f = features.begin(); f != features.end(); ++f) {
    msg.features[k].id = f->second.unique_id;
    msg.features[k].u = f->second.u;
    msg.features[k].v = f->second.v;

    k++;
  }

  _featurePublisher.publish(msg);
}

void MatcherNode::display(cv_bridge::CvImagePtr cv_ptr) {
  //Print debug image
  Mat coloredImage;
  cvtColor(cv_ptr->image, coloredImage, CV_GRAY2RGB);

  drawKeypoints(coloredImage);
  imshow(src_window, coloredImage);
  waitKey(1);
}

void MatcherNode::drawKeypoints(Mat& frame) {

  const uint32_t full_green_after_n = 20;

  const FeatureManager::FeatureMap &features = _fmanager.getFeatures();

  for (auto it = features.begin(); it != features.end(); ++it) {

    cv::Scalar drawColor(
        255
            * (1.0 - min(((double) it->second.n_obs) / full_green_after_n, 1.0)),
        255 * min(((double) it->second.n_obs) / full_green_after_n, 1.0), 0);

    Point2f pt(it->second.u, it->second.v);
    circle(frame, pt, 3, drawColor);
  }
}

MatcherNode::~MatcherNode() {

}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "libviso2_matcher");

  if (argc < 2) {
    ROS_FATAL("Two arguments needed: camera_source");
    return -1;
  }

  ros::NodeHandle n;
  string imageTopic = argv[1];

  //MatchingExtractor extractor(imageTopic, keyFramePercentage, forceExtraction, hideNew, n);
  MatcherNode node(imageTopic);

  ROS_INFO("libviso2 feature matching node started");

  ros::spin();
}

void MatcherNode::dcfgCallback(
    libviso2_matcher::matcher_parametersConfig& config, uint32_t level) {

  ROS_INFO("parameters update request");

  libviso2MatcherWrapper::parameters newparams;

  newparams.nms_n = config.nms_n;
  newparams.nms_tau = config.nms_tau;
  newparams.match_binsize = config.match_binsize;
  newparams.match_radius = config.match_radius;
  newparams.match_disp_tolerance = config.match_disp_tolerance;
  newparams.outlier_disp_tolerance = config.outlier_disp_tolerance;
  newparams.outlier_flow_tolerance = config.outlier_flow_tolerance;
  newparams.multi_stage = config.multi_stage; // these cannot be changed online, segmentation fault
  newparams.half_resolution = config.half_resolution; // these cannot be changed online, segmentation fault
  newparams.refinement = config.refinement; // these cannot be changed online, segmentation fault

  _matcher->updateParameters(newparams);

}
