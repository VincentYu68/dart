/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

/*! \mainpage Dynamic Animation and Robotics Toolkits

DART is an open source library for developing kinematic and dynamic
applications in robotics and computer animation. DART features two
main components: a multibody dynamic simulator developed by Georgia
Tech Graphics Lab and a variety of motion planning algorithms
developed by Georgia Tech Humanoid Robotics Lab. This document focuses
only on the dynamic simulator.

The multibody dynamics simulator in DART is designed to aid
development of motion control algorithms. DART uses generalized
coordinates to represent articulated rigid body systems and computes
Lagrange’s equations derived from D’Alembert’s principle to describe
the dynamics of motion. In contrast to many popular physics engines
which view the simulator as a black box, DART provides full access to
internal kinematic and dynamic quantities, such as mass matrix,
Coriolis and centrifugal force, transformation matrices and their
derivatives, etc. DART also provides efficient computation of Jacobian
matrices for arbitrary body points and coordinate frames.

The contact and collision are handled using an implicit time-stepping,
velocity-based LCP (linear-complementarity problem) to guarantee
non-penetration, directional friction, and approximated Coulombs
friction cone conditions. The LCP problem is solved efficiently by
Lemke's algorithm. For the collision detection, DART directly uses FCL
package developed by UNC Gamma Lab.

In addition, DART supports various joint types (ball-and-socket,
universal, hinge, and prismatic joints) and arbitrary meshes. DART
also provides two explicit integration methods: first-order
Runge-Kutta and fourth-order Runge Kutta.

*/

#ifndef DART_KINEMATICS_BODYNODE_H
#define DART_KINEMATICS_BODYNODE_H

#include "utils/Deprecated.h"
#include "math/LieGroup.h"

namespace dart {
namespace renderer { class RenderInterface; }
namespace kinematics {

class Dof;
class Skeleton;
class Joint;
class Shape;

/// @brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
/// Mostly automatically constructed by FileInfoSkel.
/// @see FileInfoSkel.
///
/// [Members]
/// W: world transformation (4x4 matrix)
/// J: world Jacobian (6xn matrix)
/// dJ: world Jacobian derivative (6xn matrix)
/// V: generalized body velocity (6x1 vector)
/// dV: generalized body acceleration (6x1 vector)
/// F: generalized body force (6x1 vector)
/// I: generalized body inertia (6x6 matrix)
class BodyNode
{
public:
    //--------------------------------------------------------------------------
    // Constructor and Desctructor
    //--------------------------------------------------------------------------
    /// @brief
    BodyNode();

    /// @brief
    virtual ~BodyNode();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief
    void setName(const std::string& _name) { mName = _name; }

    /// @brief
    const std::string& getName() const { return mName; }

    /// @brief
    void setVisualizationShape(Shape* _shape) { mVizShape = _shape; }

    /// @brief
    Shape* getVisualizationShape() const { return mVizShape; }

    /// @brief
    void setCollisionShape(Shape* _shape) { mColShape = _shape; }

    /// @brief
    Shape* getCollisionShape() const { return mColShape; }

    /// @brief
    void setColliding(bool _colliding) { mColliding = _colliding; }

    /// @brief
    bool getColliding() { return mColliding; }

    /// @brief
    bool getCollideState() const { return mCollidable; }

    /// @brief
    void setCollideState(bool _c) { mCollidable = _c; }

    //--------------------------------------------------------------------------
    // Kinematical Properties
    //--------------------------------------------------------------------------
    /// @brief
    void setWorldTransformation(const math::SE3& _W) { mW = _W; }

    /// @brief Transformation from the local coordinates of this body node to
    /// the world coordinates.
//    DEPRECATED Eigen::Matrix4d getWorldTransform() const
//    { return mW.getMatrix(); }
    const math::SE3& getWorldTransformation() const { return mW; }

    /// @brief Transformation from the world coordinates to the local
    /// coordinates of this body node.
//    DEPRECATED Eigen::Matrix4d getWorldInvTransform() const
//    { return mW.getInverse().getMatrix(); }
    math::SE3 getWorldInvTransformation() const { return math::Inv(mW); }

    /// @brief
    //const math::Jacobian& getJacobianBody() const { return mJacobianBody; }

    /// @brief
    ///
//    math::Jacobian getJacobianWorld() const
//    { return mJacobianBody.getAdjointed(math::SE3(mW.getOrientation())); }

    /// @brief
    const math::se3& getBodyVelocity() const { return mV; }

    /// @brief
    const math::se3& getBodyAcceleration() const { return mdV; }

    //--------------------------------------------------------------------------
    // Structueral Properties
    //--------------------------------------------------------------------------
    /// @brief
    DEPRECATED void setSkel(Skeleton* _skel) { mSkeleton = _skel; }
    void setSkeleton(Skeleton* _skel) { mSkeleton = _skel; }

    /// @brief
    DEPRECATED Skeleton* getSkel() const { return mSkeleton; }
    Skeleton* getSkeleton() const { return mSkeleton; }

    /// @brief
    void setParentJoint(Joint* _joint) { mParentJoint = _joint; }

    /// @brief
    Joint* getParentJoint() const { return mParentJoint; }

    /// @brief
    void addChildJoint(Joint* _joint);

    /// @brief
    Joint* getChildJoint(int _idx) const;

    /// @brief
    const std::vector<Joint*>& getChildJoints() const { return mJointsChild; }

    /// @brief
    int getNumChildJoints() const { return mJointsChild.size(); }

    /// @brief
    void setParentBody(BodyNode* _body) { mParentBody = _body; }

    /// @brief
    BodyNode* getParentBody() const { return mParentBody; }

    /// @brief
    void addChildBody(BodyNode* _body);

    /// @brief
    DEPRECATED BodyNode* getChildNode(int _idx) const;
    BodyNode* getChildBody(int _idx) const;

    /// @brief
    const std::vector<BodyNode*>& getChildBodies() const { return mChildBodies; }

    // TODO: Check
    void setDependDofList();

    // TODO: Check
    int getNumLocalDofs() const;

    // TODO: Check
    Dof* getDof(int _idx) const;

    // TODO: Check
    /// @brief The number of the dofs by which this node is affected.
    int getNumDependentDofs() const { return mDependentDofs.size(); }

    // TODO: Check
    /// @brief Return an dof index from the array index (< getNumDependentDofs).
    int getDependentDof(int _arrayIndex) { return mDependentDofs[_arrayIndex]; }

    //--------------------------------------------------------------------------
    // Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief (q, dq, ddq) --> (W, V, dV)
    void updateForwardKinematics(bool _firstDerivative = true,
                                 bool _secondDerivative = true);

    //--------------------------------------------------------------------------
    // Rendering
    //--------------------------------------------------------------------------
    /// @brief Render the entire subtree rooted at this body node.
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true, int _depth = 0) const;

    //--------------------------------------------------------------------------
    // Sub-functions for Recursive Kinematics Algorithms
    //--------------------------------------------------------------------------
    /// @brief
    /// parentJoint.T, parentBody.W --> W
    void _updateTransformation();

    /// @brief
    /// parentJoint.V, parentBody.V --> V
    /// parentJoint.S --> J
    void _updateVelocity(bool _updateJacobian = false);

    /// @brief
    /// parentJoint.V, parentJoint.dV, parentBody.dV, V --> dV
    /// parentJoint.dS --> dJ
    void _updateAcceleration(bool _updateJacobianDeriv = false);

protected:
    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief A unique ID of this node globally.
    int mID;

    /// @brief Counts the number of nodes globally.
    static int msBodyNodeCount;

    /// @brief
    std::string mName;

    //--------------------------------------------------------------------------
    // Properties
    //--------------------------------------------------------------------------
    /// @brief
    //std::vector<Shape*> mVizShapes;
    Shape* mVizShape;

    /// @brief
    //std::vector<Shape*> mColShapes;
    Shape* mColShape;

    /// @brief Indicating whether this node is collidable.
    bool mCollidable;

    /// @brief Whether the node is currently in collision with another node.
    bool mColliding;

    //--------------------------------------------------------------------------
    // Structual Properties
    //--------------------------------------------------------------------------
    /// @brief Pointer to the model this body node belongs to.
    Skeleton* mSkeleton;

    /// @brief
    // TODO: rename
    //Joint* mParentJoint;
    Joint* mParentJoint;

    /// @brief
    // TODO: rename
    //std::vector<Joint*> mChildJoints;
    std::vector<Joint*> mJointsChild;

    /// @brief
    BodyNode* mParentBody;

    /// @brief
    std::vector<BodyNode*> mChildBodies;

    /// @brief A list of dependent dof indices
    std::vector<int> mDependentDofs;

    //--------------------------------------------------------------------------
    // Variable Properties
    //--------------------------------------------------------------------------
    /// @brief World transformation.
    math::SE3 mW;

    /// @brief
    math::Jacobian mJacobianBody;

    /// @brief
    math::Jacobian mJacobianBodyDeriv;

    /// @brief Generalized body velocity w.r.t. body frame.
    math::se3 mV;

    /// @brief Generalized body acceleration w.r.t. body frame.
    math::se3 mdV;

private:

};

} // namespace kinematics
} // namespace dart

#endif // #ifndef DART_KINEMATICS_BODYNODE_H

