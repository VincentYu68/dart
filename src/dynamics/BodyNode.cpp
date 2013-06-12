/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "dynamics/BodyNode.h"

#include <iostream>

#include "utils/UtilsCode.h"
#include "renderer/RenderInterface.h"
#include "dynamics/Joint.h"
#include "dynamics/Shape.h"
#include "dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

int BodyNode::msBodyNodeCount = 0;

BodyNode::BodyNode(const std::string& _name)
    : mSkelIndex(-1),
      mName(_name),
      //mVizShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mVizShape(NULL),
      //mColShapes(std::vector<Shape*>(1, static_cast<Shape*>(NULL))),
      mColShape(NULL),
      mColliding(true),
      mSkeleton(NULL),
      mParentJoint(NULL),
      mJointsChild(std::vector<Joint*>(0)),
      mParentBody(NULL),
      mChildBodies(std::vector<BodyNode*>(0)),
      mW(math::SE3()),
      mV(math::se3()),
      mdV(math::se3()),
      mGravityMode(true),
      mI(math::Inertia()),
      mF(math::dse3()),
      mRestitutionCoeff(0.5),
      mFrictionCoeff(0.4)
{
    mID = BodyNode::msBodyNodeCount++;
}

BodyNode::~BodyNode()
{
}

void BodyNode::setName(const std::string& _name)
{
    mName = _name;
}

const std::string& BodyNode::getName() const
{
    return mName;
}

void BodyNode::addChildJoint(Joint* _joint)
{
    assert(_joint != NULL);

    mJointsChild.push_back(_joint);
}

Joint*BodyNode::getChildJoint(int _idx) const
{
    assert(0 <= _idx && _idx < mJointsChild.size());

    return mJointsChild[_idx];
}

void BodyNode::addChildBody(BodyNode* _body)
{
    assert(_body != NULL);

    mChildBodies.push_back(_body);
}

BodyNode*BodyNode::getChildNode(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodies.size());

    return mChildBodies[_idx];
}

BodyNode*BodyNode::getChildBody(int _idx) const
{
    assert(0 <= _idx && _idx < mChildBodies.size());

    return mChildBodies[_idx];
}

void BodyNode::setDependDofList()
{
    mDependentDofs.clear();
    if (mParentBody != NULL)
    {
        mDependentDofs.insert(mDependentDofs.end(),
                              mParentBody->mDependentDofs.begin(),
                              mParentBody->mDependentDofs.end());
    }

    for (int i = 0; i < getNumLocalDofs(); i++)
    {
        int dofID = getLocalDof(i)->getSkelIndex();
        mDependentDofs.push_back(dofID);
    }

#if _DEBUG
    for (int i = 0; i < (int)mDependentDofs.size() - 1; i++)
    {
        int now = mDependentDofs[i];
        int next = mDependentDofs[i + 1];
        if (now > next)
        {
            cerr << "Array not sorted!!!" << endl;
            exit(0);
        }
    }
#endif
}

int BodyNode::getNumLocalDofs() const
{
    return mParentJoint->getNumDofs();
}

GenCoord* BodyNode::getLocalDof(int _idx) const
{
    return mParentJoint->getDof(_idx);
}

Eigen::VectorXd BodyNode::getDependDofs() const
{
//    Eigen::VectorXd depDofs;

//    for (int i = 0; i < getNumDependentDofs(); ++i)
//    {
//        depDofs[i] = mSkeleton->get_q()[getDependentDof(i)];
//    }

    //    return depDofs;
}

math::se3 BodyNode::getVelocityWorld() const
{
    //return math::Rotate(mW, mV);
    return math::AdR(mW, mV);
}

math::se3 BodyNode::getVelocityWorldAtCOG() const
{
    math::SE3 worldFrameAtCOG = mW;
    worldFrameAtCOG.setPosition(math::Rotate(mW, -mI.getOffset()));
    return math::Ad(worldFrameAtCOG, mV);
}

math::se3 BodyNode::getVelocityWorldAtPoint(const math::Vec3& _pointBody) const
{
    math::SE3 worldFrameAtPoint = mW;
    worldFrameAtPoint.setPosition(math::Rotate(mW, -_pointBody));
    return math::Ad(worldFrameAtPoint, mV);
}

math::se3 BodyNode::getVelocityWorldAtFrame(const math::SE3& _T) const
{
    return math::Ad(math::Inv(_T) * mW, mV);
}

math::se3 BodyNode::getAccelerationWorld() const
{
    //return math::Rotate(mW, mdV);
    return math::AdR(mW, mdV);
}

math::se3 BodyNode::getAccelerationWorldAtCOG() const
{
    math::SE3 worldFrameAtCOG = mW;
    worldFrameAtCOG.setPosition(math::Rotate(mW, -mI.getOffset()));
    return math::Ad(worldFrameAtCOG, mdV);
}

math::se3 BodyNode::getAccelerationWorldAtPoint(const math::Vec3& _pointBody) const
{
    math::SE3 worldFrameAtPoint = mW;
    worldFrameAtPoint.setPosition(math::Rotate(mW, -_pointBody));
    return math::Ad(worldFrameAtPoint, mdV);
}

math::se3 BodyNode::getAccelerationWorldAtFrame(const math::SE3& _T) const
{
    return math::Ad(math::Inv(_T) * mW, mdV);
}

math::Jacobian BodyNode::getJacobianWorld() const
{
    return math::AdR(mW, mBodyJacobian);
}

math::Jacobian BodyNode::getJacobianWorldAtPoint(
        const math::Vec3& r_world) const
{
    //--------------------------------------------------------------------------
    // Jb                : body jacobian
    //
    // X = | I r_world | : frame whose origin is at contact point
    //     | 0       1 |
    //
    // X^{-1} = | I -r_world |
    //          | 0        1 |
    //
    // W = | R p |       : body frame
    //     | 0 1 |
    //
    // body_jacobian_at_contact_point = Ad(X^{-1} * W, Jb)
    //--------------------------------------------------------------------------

    return math::Ad(math::SE3(-r_world) * mW, mBodyJacobian);
}

Eigen::MatrixXd BodyNode::getJacobianWorldAtPoint_LinearPartOnly(
        const math::Vec3& r_world) const
{
    //--------------------------------------------------------------------------
    // Jb                : body jacobian
    //
    // X = | I r_world | : frame whose origin is at contact point
    //     | 0       1 |
    //
    // X^{-1} = | I -r_world |
    //          | 0        1 |
    //
    // W = | R p |       : body frame
    //     | 0 1 |
    //
    // body_jacobian_at_contact_point = Ad(X^{-1} * W, Jb)
    //--------------------------------------------------------------------------

    // TODO: Speed up here.
    Eigen::MatrixXd JcLinear = Eigen::MatrixXd::Zero(3, getNumDependentDofs());

    JcLinear = getJacobianWorldAtPoint(r_world).getEigenMatrix().bottomLeftCorner(3,getNumDependentDofs());

    return JcLinear;
}

void BodyNode::init()
{
    assert(mSkeleton != NULL);

    const int numDepDofs = getNumDependentDofs();
    mBodyJacobian.setSize(numDepDofs);
    mBodyJacobian.setZero();
    mBodyJacobianDeriv.setSize(numDepDofs);
    mBodyJacobianDeriv.setZero();

    mM = Eigen::MatrixXd::Zero(getNumDependentDofs(), getNumDependentDofs());
}

void BodyNode::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor,
                    int _depth) const
{
    if (_ri == NULL)
        return;

    _ri->pushMatrix();

    // render the self geometry
    mParentJoint->applyGLTransform(_ri);

    if (mVizShape != NULL)
    {
        _ri->pushName((unsigned)mID);
        _ri->pushMatrix();
        mVizShape->draw(_ri, _color, _useDefaultColor);
        _ri->popMatrix();
        _ri->popName();
    }

    // render the subtree
    for (unsigned int i = 0; i < mJointsChild.size(); i++)
    {
        mJointsChild[i]->getChildNode()->draw(_ri, _color, _useDefaultColor);
    }

    _ri->popMatrix();
}

void BodyNode::updateTransformation()
{
    if (mParentBody)
    {
        mW = mParentBody->getTransformationWorld()
             * mParentJoint->getLocalTransformation();
    }
    else
    {
        mW = mParentJoint->getLocalTransformation();
    }
}

void BodyNode::updateVelocity(bool _updateJacobian)
{
    //--------------------------------------------------------------------------
    // Body velocity update
    //
    // V(i) = Ad(T(i, i-1), V(i-1)) + S * dq
    //--------------------------------------------------------------------------

    if (mParentBody)
    {
        mV.setInvAd(mParentJoint->getLocalTransformation(),
                    mParentBody->getVelocityBody());
        mV += mParentJoint->getLocalVelocity();
    }
    else
    {
        mV = mParentJoint->getLocalVelocity();
    }

    if (_updateJacobian == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian update
    //
    // J = | J1 J2 ... Jn |
    //   = | Ad(T(i,i-1), J_parent) J_local |
    //
    //   J_parent: (6 x parentDOF)
    //    J_local: (6 x localDOF)
    //         Ji: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = getNumLocalDofs();
    const int numParentDOFs = getNumDependentDofs()-numLocalDOFs;

    // Parent Jacobian
    if (mParentBody != NULL)
    {
        assert(mParentBody->mBodyJacobian.getSize() + getNumLocalDofs()
               == mBodyJacobian.getSize());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            math::se3 Ji = math::InvAd(mParentJoint->getLocalTransformation(),
                                       mParentBody->mBodyJacobian[i]);
            mBodyJacobian[i] = Ji;
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobian[numParentDOFs + i] = mParentJoint->getLocalJacobian()[i];
    }
}

void BodyNode::updateEta()
{
    mEta = math::ad(mV, mParentJoint->mS*mParentJoint->get_dq()) +
           mParentJoint->mdS*mParentJoint->get_dq();
}

void BodyNode::updateAcceleration(bool _updateJacobianDeriv)
{
    // dV(i) = Ad(T(i, i-1), dV(i-1)) + ad(V(i), S * dq)
    //         + dS * dq + S * ddq
    if (mParentBody)
    {
        mdV = math::InvAd(mParentJoint->getLocalTransformation(),
                          mParentBody->getAcceleration()) +
              mEta + mParentJoint->mS*mParentJoint->get_ddq();
    }
    else
    {
        mdV = mEta + mParentJoint->mS*mParentJoint->get_ddq();
    }

    if (_updateJacobianDeriv == false)
        return;

    //--------------------------------------------------------------------------
    // Jacobian first derivative update
    //
    // dJ = | dJ1 dJ2 ... dJn |
    //   = | Ad(T(i,i-1), dJ_parent) dJ_local |
    //
    //   dJ_parent: (6 x parentDOF)
    //    dJ_local: (6 x localDOF)
    //         dJi: (6 x 1) se3
    //          n: number of dependent coordinates
    //--------------------------------------------------------------------------

    const int numLocalDOFs = getNumLocalDofs();
    const int numParentDOFs = getNumDependentDofs() - numLocalDOFs;

    // Parent Jacobian
    if (mParentBody != NULL)
    {
        assert(mParentBody->mBodyJacobianDeriv.getSize() + getNumLocalDofs()
               == mBodyJacobianDeriv.getSize());

        for (int i = 0; i < numParentDOFs; ++i)
        {
            assert(mParentJoint);
            math::se3 dJi = math::InvAd(mParentJoint->getLocalTransformation(),
                                       mParentBody->mBodyJacobianDeriv[i]);
            mBodyJacobianDeriv[i] = dJi;
        }
    }

    // Local Jacobian
    for(int i = 0; i < numLocalDOFs; i++)
    {
        mBodyJacobianDeriv[numParentDOFs + i]
                = mParentJoint->getLocalJacobianFirstDerivative()[i];
    }
}

void BodyNode::setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                               double _Ixy, double _Ixz, double _Iyz)
{
    setMomentOfInertia(_Ixx, _Iyy, _Izz, _Ixy, _Ixz, _Iyz);
}

void BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                  double _Ixy, double _Ixz, double _Iyz)
{
    mI.setAngularMomentDiag(_Ixx, _Iyy, _Izz);
    mI.setAngularMomentOffDiag(_Ixy, _Ixz, _Iyz);
}

void BodyNode::setExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext = _FextLocal;
}

void BodyNode::setExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext = _FextWorld;
}

void BodyNode::setExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
}

void BodyNode::addExtForce(const Eigen::Vector3d& _offset,
                           const Eigen::Vector3d& _force,
                           bool _isOffsetLocal, bool _isForceLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNode::addExternalForceLocal(const math::dse3& _FextLocal)
{
    mFext += _FextLocal;
}

void BodyNode::addExternalForceGlobal(const math::dse3& _FextWorld)
{
    mFext += _FextWorld;
}

void BodyNode::addExternalForceLocal(
        const Eigen::Vector3d& _posLocal,
        const Eigen::Vector3d& _linearForceGlobal)
{
    dterr << "Not implemented.\n";
}

math::dse3 BodyNode::getExternalForceGlobal() const
{
    dterr << "Not implemented.\n";
}

void BodyNode::updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces)
{
    if (mGravityMode == true)
        mFgravity = mI * math::InvAdR(mW, math::Vec3(_gravity));
    else
        mFgravity.setZero();

    mF = mI * mdV;                // Inertial force
    if (_withExternalForces)
        mF -= mFext;              // External force
    mF -= mFgravity;              // Gravity force
    mF -= math::dad(mV, mI * mV); // Coriolis force

    for (std::vector<BodyNode*>::iterator iChildBody = mChildBodies.begin();
         iChildBody != mChildBodies.end();
         ++iChildBody)
    {
        dynamics::Joint* childJoint = (*iChildBody)->getParentJoint();
        assert(childJoint != NULL);
        BodyNode* bodyDyn = dynamic_cast<BodyNode*>(*iChildBody);
        assert(bodyDyn != NULL);

        mF += math::InvdAd(childJoint->getLocalTransformation(),
                           bodyDyn->getBodyForce());
    }
}

void BodyNode::updateGeneralizedForce(bool _withDampingForces)
{
    assert(mParentJoint != NULL);

    const math::Jacobian& J = mParentJoint->getLocalJacobian();

//    if (_withDampingForces)
//        mF -= mFDamp;

    mParentJoint->set_tau(J.getInnerProduct(mF));
}

void BodyNode::updateArticulatedInertia()
{
    // TODO: Need code optimization
    mAI = mI;

    for (std::vector<Joint*>::iterator itrJoint = mJointsChild.begin();
         itrJoint != mJointsChild.end();
         ++itrJoint)
    {
        mAI += (*itrJoint)->getChildBody()->mPi.Transform(
                   math::Inv((*itrJoint)->getLocalTransformation()));
    }
}

void BodyNode::updateBiasForce(const Eigen::Vector3d& _gravity)
{
    if (mGravityMode == true)
        mFgravity = mI * math::InvAdR(mW, math::Vec3(_gravity));
    else
        mFgravity.setZero();

    mB = -math::dad(mV, mI*mV);
    mB -= mFext;
    mB -= mFgravity;

    for (std::vector<Joint*>::iterator itrJoint = mJointsChild.begin();
         itrJoint != mJointsChild.end();
         ++itrJoint)
    {
        mB += math::InvdAd((*itrJoint)->getLocalTransformation(),
                           (*itrJoint)->getChildBody()->mBeta);
    }
}

void BodyNode::updatePsi()
{
    // TODO: Need code optimization
    assert(mParentJoint != NULL);

    int n = mParentJoint->getDOF();
    const math::Jacobian& S = mParentJoint->mS;
    mAI_S = Eigen::MatrixXd::Zero(6, n);
    mPsi = Eigen::MatrixXd::Zero(n, n);

    for (int i = 0; i < n; ++i)
    {
        mAI_S.col(i) = (mAI * S[i]).getEigenVector();
        for (int j = 0; j < n; ++j)
        {
            mPsi(i, j) = math::Inner(S[j], mAI * S[i]);
        }
    }

    mPsi = mPsi.inverse();
}

void BodyNode::updatePi()
{
    // TODO:
    mPi = -(mAI_S * mPsi * mAI_S.transpose());
    mPi += mAI;
}

void BodyNode::updateBeta()
{
    // TODO: Optimization later...
    //       Mystery code is here...
    int n = mParentJoint->getDOF();
    const math::Jacobian& S = mParentJoint->mS;
    mPsi;
    mParentJoint->get_tau();
    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd Psi_tau = Eigen::VectorXd::Zero(n);

    for (int i = 0; i < n; ++i)
    {
        tmp[i] = math::Inner(S[i], mAI * mEta + mB);
    }
    Psi_tau = mPsi*(mParentJoint->get_tau() - tmp);

    math::se3 newS;

    for (int i = 0; i < n; ++i)
    {
        newS += S[i]*Psi_tau[i];
    }

    mBeta = mB + mAI * (mEta + newS);
}

void BodyNode::update_ddq()
{
    int n = mParentJoint->getDOF();
    const math::Jacobian& S = mParentJoint->mS;
    Eigen::VectorXd tmp2 = Eigen::VectorXd::Zero(n);
    math::se3 tmp;

    if (mParentBody) {
        tmp = math::InvAd(mParentJoint->getLocalTransformation(),
                                    mParentBody->getAcceleration() + mEta);
    }
    else
    {
        tmp = math::InvAd(mParentJoint->getLocalTransformation(), mEta);
    }

    for (int i = 0; i < n; ++i)
    {
        tmp2[i] = math::Inner(S[i], mAI*tmp + mB);
    }

    Eigen::VectorXd ddq = mPsi * (mParentJoint->get_tau() - tmp2);
    mParentJoint->set_ddq(ddq);
}

void BodyNode::update_F_fs()
{
    mF = mAI*mdV + mB;
}

void BodyNode::updateDampingForce()
{
    dterr << "Not implemented.\n";
}

void BodyNode::updateMassMatrix()
{
    mM.triangularView<Eigen::Upper>() = mBodyJacobian.getEigenMatrix().transpose() * mI.toTensor() * mBodyJacobian.getEigenMatrix();
    mM.triangularView<Eigen::StrictlyLower>() = mM.transpose();
}

void BodyNode::aggregateMass(Eigen::MatrixXd& _M)
{
    for(int i=0; i<getNumDependentDofs(); i++)
    {
        for(int j=0; j<getNumDependentDofs(); j++)
        {
            _M(mDependentDofs[i], mDependentDofs[j]) += mM(i, j);
        }
    }
}

void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
    dterr << "Not implemented.\n";
}

void BodyNode::clearExternalForces()
{
    mFext.setZero();
    dterr << "Not implemented.\n";
}


} // namespace dynamics
} // namespace dart

