/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 08/22/2013
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

#include "UniversalJoint.h"

#include "math/UtilsMath.h"
#include "math/LieGroup.h"
#include "math/UtilsRotation.h"

namespace dart {
namespace dynamics {

UniversalJoint::UniversalJoint(const math::Axis& _axis0,
                               const math::Axis& _axis1)
    : Joint("Universal joint")
{
    mJointType = UNIVERSAL;

    mGenCoords.push_back(&mCoordinate[0]);
    mGenCoords.push_back(&mCoordinate[1]);

    mS = Eigen::Matrix<double,6,2>::Zero();
    mdS = Eigen::Matrix<double,6,2>::Zero();

    mDampingCoefficient.resize(2, 0);

    mAxis[0] = _axis0;
    mAxis[1] = _axis1;
}

UniversalJoint::~UniversalJoint()
{
}

void UniversalJoint::setAxis(int _idx, const math::so3 &_axis)
{
    assert(_axis.norm() == 1);
    assert(0 <= _idx && _idx <=1);

    mAxis[_idx] = _axis;
}

const math::so3 &UniversalJoint::getAxis(int _idx) const
{
    assert(0 <= _idx && _idx <=1);

    return mAxis[_idx];
}

inline void UniversalJoint::_updateTransformation()
{
    // T
    mT = mT_ParentBodyToJoint *
         math::ExpAngular(mAxis[0] * mCoordinate[0].get_q()) *
         math::ExpAngular(mAxis[1] * mCoordinate[1].get_q()) *
         math::Inv(mT_ChildBodyToJoint);
}

inline void UniversalJoint::_updateVelocity()
{
    // S
    mS = math::AdTAngular(mT_ChildBodyToJoint, mAxis[0]);

    // V = S * dq
    mV = mS * get_dq();
    //mV.setAngular(mAxis * mCoordinate.get_q());
}

inline void UniversalJoint::_updateAcceleration()
{
    // dS = 0
    mdS.setZero();

    // dV = dS * dq + S * ddq
    mdV = mS * get_ddq();
}

} // namespace dynamics
} // namespace dart
