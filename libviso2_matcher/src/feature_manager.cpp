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
 * feature_manager.cpp
 *
 *  Created on: Jan 28, 2015
 *      Author: davide
 */

#include "feature_manager.h"

using namespace std;

FeatureManager::FeatureManager() : _max_id(0) {
  _features = new FeatureMap;
}

void FeatureManager::updateFeatures(
    const std::vector<libviso2MatcherWrapper::p_match>& matches) {

  FeatureMap *newFeatures = new FeatureMap;

  for (auto m = matches.begin(); m != matches.end(); ++m) {
    auto f = _features->find(m->i1p);

    Feature newf;

    newf.u = m->u1c;
    newf.v = m->v1c;

    if (f != _features->end()) {
      newf.n_obs = f->second.n_obs + 1;
      newf.unique_id = f->second.unique_id;
    } else {
      newf.n_obs = 1;
      newf.unique_id = ++_max_id;
    }

    (*newFeatures)[m->i1c] =  newf;
  }

  delete _features;
  _features = newFeatures;
}

const FeatureManager::FeatureMap& FeatureManager::getFeatures() const {
  return *_features;
}
