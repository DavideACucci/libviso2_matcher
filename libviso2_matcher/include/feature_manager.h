/*
 * feature_manager,
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
 * feature_manager.h
 *
 *  Created on: Jan 28, 2015
 *      Author: davide
 */

#ifndef SRC_FEATURE_MANAGER_H_
#define SRC_FEATURE_MANAGER_H_

#include "libviso2_matcher_wrapper.h"

class FeatureManager {

public:
  struct Feature_s {
    uint32_t n_obs;
    uint32_t u,v;
    uint64_t unique_id;
  };

  typedef Feature_s Feature;
  typedef std::map<int32_t, Feature> FeatureMap;

  FeatureManager();

  void updateFeatures(const std::vector<libviso2MatcherWrapper::p_match> &matches);

  const FeatureMap &getFeatures() const;

protected:
  FeatureMap *_features;

  uint64_t _max_id;

};

#endif /* SRC_FEATURE_MANAGER_H_ */
