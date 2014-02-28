/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dynamics/FemSkeleton.h"

#include <string>
#include <vector>

#include <dart/dynamics/Joint.h>

#include "dart/dynamics/FemPoint.h"
#include "dart/dynamics/FemBodyNode.h"

namespace dart {
namespace dynamics {

FEMSkeleton::FEMSkeleton(const std::string& _name)
  : Skeleton(_name)
{
}

FEMSkeleton::~FEMSkeleton()
{
}

void FEMSkeleton::addFEMBodyNode(FEMBodyNode* _body)
{
  assert(_body && _body->getParentJoint());
  mFEMBodyNodes.push_back(_body);

  Skeleton::addBodyNode(_body);
}

FEMBodyNode* FEMSkeleton::getFEMBodyNode(int _idx) const
{
  assert(0 <= _idx && _idx < mFEMBodyNodes.size());
  return mFEMBodyNodes[_idx];
}

int FEMSkeleton::getNumFEMBodyNodes() const
{
  return mFEMBodyNodes.size();
}

int FEMSkeleton::getNumRigidBodyNodes() const
{
  return mBodyNodes.size() - mFEMBodyNodes.size();
}

FEMBodyNode* FEMSkeleton::getFEMBodyNode(const std::string& _name) const
{
  assert(!_name.empty());
  for (std::vector<FEMBodyNode*>::const_iterator itrFEMBodyNode =
       mFEMBodyNodes.begin();
       itrFEMBodyNode != mFEMBodyNodes.end(); ++itrFEMBodyNode)
  {
    if ((*itrFEMBodyNode)->getName() == _name)
      return *itrFEMBodyNode;
  }
  return NULL;
}

void FEMSkeleton::init(double _timeStep, const Eigen::Vector3d& _gravity)
{
  // Initialize this skeleton.
  Skeleton::init(_timeStep, _gravity);
}

    void FEMSkeleton::gatherFemSim(std::vector<simulation::FemSimulation*>& femsim) {
        for (int i = 0; i < mFEMBodyNodes.size() ; i ++) {
            femsim.push_back(mFEMBodyNodes[i]->getFEMSim());
        }
    }

}  // namespace dynamics
}  // namespace dart

