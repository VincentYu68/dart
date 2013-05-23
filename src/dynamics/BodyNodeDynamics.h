/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
#define DART_DYNAMICS_BODYNODE_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

#include "kinematics/BodyNode.h"
#include "dynamics/Inertia.h"

namespace dart {
namespace dynamics {

/// @brief BodyNodeDynamics class represents a single node of the skeleton for
/// dynamics.
class BodyNodeDynamics : public kinematics::BodyNode
{
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTORS AND DESTRUCTOR
    //--------------------------------------------------------------------------
    /// @brief Default constructor.
    BodyNodeDynamics();

    /// @brief Default destructor
    virtual ~BodyNodeDynamics();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setGravityMode(bool _onoff) { mGravityMode = _onoff; }

    /// @brief
    bool getGravityMode() const { return mGravityMode; }

protected:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// TODO: Not implemented yet!
    bool mGravityMode;

    /// @brief Generalized inertia.
    dynamics::Inertia mI;

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief Generalized body force w.r.t. body frame.
    math::dse3 mF;
private:

public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_BODYNODE_DYNAMICS_H
