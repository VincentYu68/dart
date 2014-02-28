/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/simulation/FemWorld.h"
#include "dart/dynamics/FemSkeleton.h"
#include "dart/simulation/FemSim.h"
#include "dart/constraint/SoftConstraintDynamics.h"
#include "dart/collision/fcl_mesh/SoftFCLMeshCollisionDetector.h"

namespace dart {
namespace simulation {

FEMWorld::FEMWorld()
  : World()
{
  // TODO(JS): Temporary code
  delete mConstraintHandler;

  mConstraintHandler = new constraint::SoftConstraintDynamics(mSkeletons,
                                                              mTimeStep);

  mConstraintHandler->setCollisionDetector(
        new collision::SoftFCLMeshCollisionDetector());
}

FEMWorld::~FEMWorld()
{
}

    void FEMWorld::addFemSim(FemSimulation* femsim) {
        _femSims.push_back(femsim);
    }
    
    void FEMWorld::gatherFemSim() {
        for (int i = 0; i < _femSims.size(); i ++) {
            delete _femSims[i];
        }
        
        _femSims.clear();
        
        for (std::vector<dynamics::Skeleton*>::iterator itr = mSkeletons.begin();
             itr != mSkeletons.end(); ++itr) {
            (dynamic_cast<dart::dynamics::FEMSkeleton*>(*itr))->gatherFemSim(_femSims);
        }
    }
    
    void FEMWorld::step() {
        // compute forward dynamics
        for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
             it != mSkeletons.end(); ++it) {
            (*it)->computeForwardDynamics();
        }
        
        for (int i = 0; i < _femSims.size(); i ++) {
            _femSims[i]->step(mTimeStep);
        }
        
        for (std::vector<dynamics::Skeleton*>::iterator itr = mSkeletons.begin();
             itr != mSkeletons.end(); ++itr) {
            (*itr)->clearInternalForceVector();
            (*itr)->clearExternalForceVector();
        }
        
        mTime += mTimeStep;
        mFrame++;
    }
    
    
    
}  // namespace simulation
}  // namespace dart
