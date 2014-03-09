/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/FemPoint.h"

#include <dart/common/Console.h>
#include <dart/math/Geometry.h>
#include <dart/math/Helpers.h>
#include <dart/dynamics/EllipsoidShape.h>
#include <dart/renderer/RenderInterface.h>

#include "dart/dynamics/FemBodyNode.h"

namespace dart {
namespace dynamics {

FEMPoint::FEMPoint(FEMBodyNode* _femBodyNode)
  : GenCoordSystem(),
    mMass(0.0005),
    mX(Eigen::Vector3d::Zero()),
    mX0(Eigen::Vector3d::Zero()),
    mParentFEMBodyNode(_femBodyNode),
    mFext(Eigen::Vector3d::Zero()),
    mForce(Eigen::Vector3d::Zero()),
    mShape(new EllipsoidShape(Eigen::Vector3d(0.01, 0.01, 0.01))),
    mIsColliding(false),
    mGravity(Eigen::Vector3d(0, -9.8, 0))
{
  assert(mParentSoftBodyNode != NULL);

  mGenCoords.push_back(&mCoordinate[0]);
  mGenCoords.push_back(&mCoordinate[1]);
  mGenCoords.push_back(&mCoordinate[2]);
    
    immobile = false;
}

FEMPoint::~FEMPoint()
{
  delete mShape;
}

void FEMPoint::setMass(double _mass)
{
  assert(0.0 < _mass);
  mMass = _mass;
}

double FEMPoint::getMass() const
{
  return mMass;
}

void FEMPoint::setColliding(bool _isColliding)
{
  mIsColliding = _isColliding;
}

bool FEMPoint::isColliding()
{
  return mIsColliding;
}

    void FEMPoint::setImmobile(bool i)
    {
        immobile = i;
    }

    
    bool FEMPoint::isImmobile() {
        return immobile;
    }
    
void FEMPoint::addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal)
{
    if (immobile) return;
    
    mFext += _force;
    
}
    
    Eigen::Vector3d FEMPoint::getExtForce() {
        return mFext;
    }

void FEMPoint::clearExtForce()
{
  mFext.setZero();
}
    
void FEMPoint::setRestingPosition(const Eigen::Vector3d& _p)
{
  mX0 = _p;
    set_q(mX0);
}

    

const Eigen::Vector3d& FEMPoint::getRestingPosition() const
{
  return mX0;
}

const Eigen::Vector3d FEMPoint::getPosition() const
{
  return get_q();
}

FEMBodyNode* FEMPoint::getParentFEMBodyNode() const
{
  return mParentFEMBodyNode;
}

int FEMPoint::getNumDependentGenCoords() const
{
  return mDependentGenCoordIndices.size();
}

int FEMPoint::getDependentGenCoord(int _arrayIndex) const
{
  assert(0 <= _arrayIndex && _arrayIndex < mDependentGenCoordIndices.size());
  return mDependentGenCoordIndices[_arrayIndex];
}

void FEMPoint::init()
{
  // Dependen generalized coordinate setting
  int parentDof = mParentFEMBodyNode->getNumDependentGenCoords();

  mDependentGenCoordIndices.resize(parentDof + 3);
  for (int i = 0; i < parentDof; ++i)
  {
    mDependentGenCoordIndices[i]
            = mParentFEMBodyNode->getDependentGenCoordIndex(i);
  }

  mDependentGenCoordIndices[parentDof]     = mCoordinate[0].getSkeletonIndex();
  mDependentGenCoordIndices[parentDof + 1] = mCoordinate[1].getSkeletonIndex();
  mDependentGenCoordIndices[parentDof + 2] = mCoordinate[2].getSkeletonIndex();
}

void FEMPoint::update_ddq()
{
    if (immobile) return;
    
    //std::cout << mFext/mMass << "\n\n";
  // ddq = imp_psi*(alpha - m*(dw(parent) x mX + dv(parent))
    mForce = mFext;
    Eigen::Vector3d ddq = (mFext/mMass/* + Eigen::Vector3d(0, -9.8, 0)*/);
    set_ddq(ddq);
    
    //Eigen::Vector3d dq = get_dq();
    //set_dq(0.99*dq);
    
  assert(!math::isNan(ddq));
}

void FEMPoint::draw(renderer::RenderInterface* _ri,
                     const Eigen::Vector4d& _color,
                     bool _useDefaultColor) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
//  mParentJoint->applyGLTransform(_ri);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = get_q();
  _ri->transform(T);
  Eigen::Vector4d color1;
  color1 << 0.8, 0.3, 0.3, 1.0;
  mShape->draw(_ri, color1, false);
  _ri->popMatrix();

//  _ri->pushName((unsigned)mID);
  _ri->pushMatrix();
  T.translation() = mX0;
  _ri->transform(T);
  Eigen::Vector4d color2;
  color2 << 0.3, 0.8, 0.3, 1.0;
  mShape->draw(_ri, color2, false);
  _ri->popMatrix();
//  _ri->popName();
}
    
    void FEMPoint::setGravity(const Eigen::Vector3d& _g) {
        //mGravity = _g;
    }

    /*void FEMPoint::addConnectedPoint(FEMPoint* p) {
        if (mConnectedPoints.size() == 3) {
            mConnectedPoints.erase(mConnectedPoints.begin()+1);
        }
        mConnectedPoints.push_back(p);
    }
    
    void FEMPoint::preCompute() {
        if (mConnectedPoints.size() < 3) {
            std::cout<<"Connected point assignment error.\n";
            return;
        }
        Eigen::Matrix3d temp = computeOrthCoord(mConnectedPoints[0]->getRestingPosition() - getRestingPosition(),
                                                mConnectedPoints[1]->getRestingPosition() - getRestingPosition(),
                                                mConnectedPoints[2]->getRestingPosition() - getRestingPosition());
        _mN = temp.transpose();
    }
    
    Eigen::Matrix3d FEMPoint::computeOrthCoord(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3)
    {
        v1.normalize(); v2.normalize(); v3.normalize();
        Eigen::Vector3d n1, n2, n3;
        n1 = (v1+v2+v3); n1.normalize();
        n2 = v1.cross(n1); n2.normalize();
        n3 = n1.cross(n2); n3.normalize();
        
        Eigen::Matrix3d rst;
        rst << n1, n2, n3;
        return rst;
    }
    
    void FEMPoint::updateRotationMatrix() {
        Eigen::Matrix3d Ns = computeOrthCoord(mConnectedPoints[0]->get_q()-get_q(),
                                            mConnectedPoints[1]->get_q() - get_q(),
                                           mConnectedPoints[2]->get_q() - get_q());
        // std::cout << Ns << std::endl;
        _mR = Ns*_mN;
    }
    
    Eigen::Matrix3d FEMPoint::getRotationMatrix() {
        return _mR;
    }*/
    
}  // namespace dynamics
}  // namespace dart





