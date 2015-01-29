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
 * libviso2_matcher_wrapper.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Davide A. Cucci
 */

#ifndef SRC_LIBVISO2MATCHERWRAPPER_H_
#define SRC_LIBVISO2MATCHERWRAPPER_H_

#include <opencv2/opencv.hpp>

#include "libviso2/matcher.h"

/**
 * this class extends the libviso2 matcher
 *
 * - adds methods for dealing with opencv images
 *
 */

class libviso2MatcherWrapper : public Matcher {

public:

  libviso2MatcherWrapper(Matcher::parameters &params);

  void pushBack(cv::Mat &img, bool repleace);

};

#endif /* SRC_LIBVISO2MATCHERWRAPPER_H_ */
