/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/21/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "math/LieGroup.h"
#include "kinematics/RevoluteJoint.h"

namespace dart {

using namespace math;

namespace kinematics {

RevoluteJoint::RevoluteJoint()
    : Joint(),
      mAxis(math::so3(1.0, 0.0, 0.0))
{
    mName.assign("Revolute joint");
    mJointType = REVOLUTE;
    mDofs.push_back(&mCoordinate);
    mS.setSize(1);
    mdS.setSize(1);
}

RevoluteJoint::~RevoluteJoint()
{
}

void RevoluteJoint::_updateTransformation()
{
    // T
    mT = mT_ParentBodyToJoint
         * Exp(se3(mAxis * mCoordinate.get_q(), Vec3(0.0, 0.0, 0.0)))
         * Inv(mT_ChildBodyToJoint);
}

void RevoluteJoint::_updateVelocity()
{
    // S
    mS.setColumn(0, math::Ad(mT_ChildBodyToJoint, math::se3(mAxis, Vec3(0.0, 0.0, 0.0))));

    // V = S * dq
    mV = mS * get_dq();
    //mV.setAngular(mAxis * mCoordinate.get_q());
}

void RevoluteJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace kinematics
} // namespace dart
