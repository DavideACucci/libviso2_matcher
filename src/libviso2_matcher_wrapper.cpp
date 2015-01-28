/*
 * libviso2_matcher_wrapper,
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
 * libviso2_matcher_wrapper.cpp
 *
 *  Created on: Jan 28, 2015
 *      Author: davide
 */

#include "../include/libviso2_matcher_wrapper.h"

libviso2MatcherWrapper::libviso2MatcherWrapper(Matcher::parameters& params) : Matcher(params) {
}

void libviso2MatcherWrapper::pushBack(cv::Mat& img, bool replace) {

  // we cannot handle directly each type of opencv image
  assert( (img.flags & cv::Mat::CONTINUOUS_FLAG) && img.depth() == CV_8U && img.channels() == 1 );

  int32_t dim[] = {img.cols, img.rows, img.cols};

  Matcher::pushBack(img.data, dim, replace);

}
