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

#include "math/LieGroup.h"
#include "math/UtilsMath.h"
#include "dynamics/BodyNode.h"
#include "dynamics/Dof.h"
#include "dynamics/Joint.h"
#include "dynamics/Skeleton.h"
#include "dynamics/Skeleton.h"

using namespace std;
using namespace Eigen;

namespace dart {
namespace dynamics {

Skeleton::Skeleton(const std::string& _name)
    : GenCoordSystem(),
      mName(_name),
      mSelfCollidable(false),
      mTotalMass(0.0),
      mImmobile(false),
      mJointLimit(true),
      mFrame(math::SE3::Identity())
{
}

Skeleton::~Skeleton()
{
}

void Skeleton::setWorldTransformation(const math::SE3& _W, bool _updateChilds)
{
    mFrame = _W;

    if (_updateChilds)
    {
        updateForwardKinematics(false, false);
    }
}

void Skeleton::initDynamics()
{
    initKinematics();

    int DOF = getDOF();

    mM    = MatrixXd::Zero(DOF, DOF);
    mMInv = MatrixXd::Zero(DOF, DOF);
    mC    = MatrixXd::Zero(DOF, DOF);
    mCvec = VectorXd::Zero(DOF);
    mG    = VectorXd::Zero(DOF);
    mCg   = VectorXd::Zero(DOF);
    set_tau(VectorXd::Zero(DOF));
    mFext = VectorXd::Zero(DOF);
    mFc   = VectorXd::Zero(DOF);

    // calculate mass
    // init the dependsOnDof stucture for each bodylink
    mTotalMass = 0.0;
    for(int i = 0; i < getNumBodyNodes(); i++)
        mTotalMass += getBodyNode(i)->getMass();
}

void Skeleton::addBodyNode(BodyNode* _body, bool _addParentJoint)
{
    assert(_body != NULL);

    mBodyNodes.push_back(_body);
    _body->setSkelIndex(mBodyNodes.size() - 1);

    // The parent joint possibly be null
    if (_addParentJoint)
        addJoint(_body->getParentJoint());
}

void Skeleton::addJoint(Joint* _joint)
{
    assert(_joint);

    mJoints.push_back(_joint);
    _joint->setSkelIndex(mJoints.size() - 1);

    const std::vector<GenCoord*>& dofs = _joint->getGenCoords();
    for (std::vector<GenCoord*>::const_iterator itrDof = dofs.begin();
         itrDof != dofs.end();
         ++itrDof) {
        mGenCoords.push_back((*itrDof));
        (*itrDof)->setSkelIndex(mGenCoords.size() - 1);
    }
}

BodyNode*Skeleton::findBodyNode(const string& _name) const
{
    assert(!_name.empty());

    for (std::vector<BodyNode*>::const_iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody) {
        if ((*itrBody)->getName() == _name)
            return *itrBody;
    }

    return NULL;
}

//Eigen::VectorXd Skeleton::getDependentConfiguration(BodyNode* _beginBody,
//                                                    BodyNode* _endBody) const
//{
//    assert(_beginBody != NULL);
//    assert(_endBody != NULL);
//
//    int beginBodyID = _beginBody->getSkelIndex();
//    int endBodyID = _endBody->getSkelIndex();
//
//
//}

void Skeleton::setPose(const Eigen::VectorXd& _pose,
                       bool bCalcTrans,
                       bool bCalcDeriv)
{
    for (int i = 0; i < getDOF(); i++)
        mGenCoords.at(i)->set_q(_pose[i]);

    if (bCalcTrans) {
        if (bCalcDeriv)
            updateForwardKinematics(true, false);
        else
            updateForwardKinematics(false, false);
    }
}

void Skeleton::initKinematics()
{
    mRootBodyNode = mBodyNodes[0];
    mToRootBody = math::Inv(mFrame) * mRootBodyNode->getWorldInvTransform();

    // calculate mass
    // init the dependsOnDof stucture for each bodylink
    for(int i = 0; i < getNumBodyNodes(); i++)
    {
        mBodyNodes.at(i)->setSkel(this);
        mBodyNodes.at(i)->setDependDofList();
        mBodyNodes.at(i)->init();
    }

    //boost::write_graphviz(std::cout, *mGraph);
}

void Skeleton::updateForwardKinematics(bool _firstDerivative,
                                       bool _secondDerivative)
{
    _updateJointKinematics(_firstDerivative, _secondDerivative);
    _updateBodyForwardKinematics(_firstDerivative, _secondDerivative);
}

void Skeleton::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const
{
    mRootBodyNode->draw(_ri, _color, _useDefaultColor);
}

void Skeleton::_updateJointKinematics(bool _firstDerivative,
                                      bool _secondDerivative)
{
    for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
         itrJoint != mJoints.end(); ++itrJoint)
    {
        (*itrJoint)->updateKinematics(_firstDerivative,
                                      _secondDerivative);
    }
}

void Skeleton::_updateBodyForwardKinematics(bool _firstDerivative,
                                            bool _secondDerivative)
{
    for (std::vector<BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end(); ++itrBody)
    {
        (*itrBody)->updateTransformation();

        if (_firstDerivative)
            (*itrBody)->updateVelocity();

        if (_secondDerivative)
        {
            (*itrBody)->updateEta();
            (*itrBody)->updateAcceleration();
        }
    }

    mFrame = mRootBodyNode->getWorldTransform() * math::Inv(mToRootBody);
}

void Skeleton::computeInverseDynamics(const Eigen::Vector3d& _gravity,
                                      bool _computeJacobian,
                                      bool _computeJacobianDeriv,
                                      bool _withExternalForces,
                                      bool _withDampingForces)
{
    _updateJointKinematics();

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody) {
        (*itrBody)->updateTransformation();
        (*itrBody)->updateVelocity(_computeJacobian);
        (*itrBody)->updateEta();
        (*itrBody)->updateAcceleration(_computeJacobianDeriv);
    }

    // Backward recursion
    for (std::vector<dynamics::BodyNode*>::reverse_iterator ritrBody
         = mBodyNodes.rbegin();
         ritrBody != mBodyNodes.rend();
         ++ritrBody)
    {
        (*ritrBody)->updateBodyForce(_gravity,
                                     _withExternalForces);
        (*ritrBody)->updateGeneralizedForce(_withDampingForces);
    }
}

void Skeleton::evalExternalForces()
{
    mFext.setZero();
    int nNodes = getNumBodyNodes();

    // recursive from child to parent
    for (int i = nNodes-1; i >= 0; i--)
    {
        BodyNode* nodei = getBodyNode(i);

        nodei->evalExternalForcesRecursive(mFext);
    }
}

void Skeleton::clearExternalForces()
{
    int nNodes = getNumBodyNodes();

    for (int i = 0; i < nNodes; i++)
        mBodyNodes.at(i)->clearExternalForces();
}

void Skeleton::computeInverseDynamicsWithZeroAcceleration(
        const Eigen::Vector3d& _gravity, bool _withExternalForces)
{

}

void Skeleton::computeForwardDynamics(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void Skeleton::computeForwardDynamicsID(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    // TODO: add evaluation of external forces in generalized coordinate.
    evalExternalForces();

    Eigen::VectorXd qddot = this->getInvMassMatrix()
                            * (-this->getCombinedVector()
                               + this->getExternalForces()
                               + this->getInternalForces()
                               + this->getDampingForces()
                               + this->getConstraintForces() );

    this->set_ddq(qddot);
}

void Skeleton::computeForwardDynamicsID2(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    int n = getDOF();

    // skip immobile objects in forward simulation
    if (getImmobileState() == true || n == 0)
    {
        return;
    }

    // Save current tau
    Eigen::VectorXd tau_old = get_tau();

    // Set ddq as zero
    set_ddq(Eigen::VectorXd::Zero(n));

    //
    mM = Eigen::MatrixXd::Zero(n,n);

    // M(q) * ddq + b(q,dq) = tau
    computeInverseDynamics(_gravity);
    Eigen::VectorXd b = get_tau();
    mCg = b;

    // Calcualtion M column by column
    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorXd basis = Eigen::VectorXd::Zero(n);
        basis(i) = 1;
        set_ddq(basis);
        computeInverseDynamics(_gravity);
        mM.col(i) = get_tau() - b;
    }

    // Restore the torque
    set_tau(tau_old);

    // TODO: add evaluation of external forces in generalized coordinate.
    evalExternalForces();

    Eigen::VectorXd qddot = this->getInvMassMatrix()
                            * (-this->getCombinedVector()
                               + this->getExternalForces()
                               + this->getInternalForces()
                               + this->getDampingForces()
                               + this->getConstraintForces() );

    //mMInv = mM.inverse();
    mMInv = mM.ldlt().solve(MatrixXd::Identity(n,n));

    this->set_ddq(qddot);
}

void Skeleton::computeForwardDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    int n = getDOF();

    // skip immobile objects in forward simulation
    if (getImmobileState() == true || n == 0)
    {
        return;
    }

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody)
    {
        (*itrBody)->updateTransformation();
        (*itrBody)->updateVelocity();
        (*itrBody)->updateEta();
    }

    // Backward recursion
    for (std::vector<dynamics::BodyNode*>::reverse_iterator ritrBody
         = mBodyNodes.rbegin();
         ritrBody != mBodyNodes.rend();
         ++ritrBody)
    {
        (*ritrBody)->updateArticulatedInertia();
        (*ritrBody)->updateBiasForce(_gravity);
        (*ritrBody)->updatePsi();
        (*ritrBody)->updatePi();
        (*ritrBody)->updateBeta();
    }

    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody)
    {
        (*itrBody)->update_ddq();
        (*itrBody)->updateAcceleration();
        (*itrBody)->update_F_fs();
     }
}

void Skeleton::computeHybridDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
}

void Skeleton::computeImpuseBasedForwardDynamics(const Eigen::Vector3d& _gravity,
                                                 bool _equationsOfMotion)
{

}

void Skeleton::computeImpuseBasedHybridDynamics(const Eigen::Vector3d& _gravity,
                                                bool _equationsOfMotion)
{

}

void Skeleton::computeEquationsOfMotionID(
        const Eigen::Vector3d& _gravity)
{
    int n = getDOF();

    // skip immobile objects in forward simulation
    if (getImmobileState() == true || n == 0)
    {
        return;
    }

    // Save current tau
    Eigen::VectorXd tau_old = get_tau();

    // Set ddq as zero
    set_ddq(Eigen::VectorXd::Zero(n));

    // M(q) * ddq + b(q,dq) = tau
    computeInverseDynamics(_gravity, true);
    mCg = get_tau();

    // Calcualtion mass matrix, M
    mM = Eigen::MatrixXd::Zero(n,n);
    for (int i = 0; i < getNumBodyNodes(); i++)
    {
        BodyNode *nodei = static_cast<BodyNode*>(getBodyNode(i));
        nodei->updateMassMatrix();
        nodei->aggregateMass(mM);
    }

    // Inverse of mass matrix
    mMInv = mM.ldlt().solve(MatrixXd::Identity(n,n));

    // Restore the torque
    set_tau(tau_old);
}

void Skeleton::computeEquationsOfMotionFS(
        const Eigen::Vector3d& _gravity)
{
}

Eigen::VectorXd Skeleton::getDampingForces() const
{
    Eigen::VectorXd dampingForce = Eigen::VectorXd(getDOF());

    int idx = 0;
    int numJoints = mJoints.size();

    for (int i = 0; i < numJoints; ++i) {
        int numDofsJoint = mJoints[i]->getDOF();
        Eigen::VectorXd dampingForceJoint = mJoints[i]->getDampingForce();

        dampingForce.segment(idx, numDofsJoint) = dampingForceJoint;

        idx += numDofsJoint;
    }

//    if (getDOF() > 0)
//    for (int i = 0; i < 6; ++i) {
//        if (dampingForce[i] != 0.0)
//            int a = 10;
//    }

    return dampingForce;
}

double Skeleton::getKineticEnergy() const
{
    double KineticEnergy = 0.0;

    for (int i = 0; i < mBodyNodes.size(); i++)
        KineticEnergy += mBodyNodes[i]->getKineticEnergy();

    return 0.5 * KineticEnergy;
}

double Skeleton::getPotentialEnergy() const
{
    double potentialEnergy = 0.0;

    //// Gravity and Springs on bodies
    //for (int i = 0; i < mBodies.size(); i++)
    //    potentialEnergy += mBodies[i]->getPotentialEnergy();

    // Springs on joints
    for (int i = 0; i < mJoints.size(); i++)
        potentialEnergy += mJoints[i]->getPotentialEnergy();

    return potentialEnergy;
}

math::Vec3 Skeleton::getPositionCOMGlobal()
{
    math::Vec3 p(0,0,0);

    // TODO: Not implemented.

    return p;
}

math::Vec3 Skeleton::getVelocityCOMGlobal()
{
    math::Vec3 p(0,0,0);

    // TODO: Not implemented.

    return p;
}

math::Vec3 Skeleton::getAccelerationCOMGlobal()
{
    math::Vec3 p(0,0,0);

    // TODO: Not implemented.

    return p;
}

math::dse3 Skeleton::getMomentumGlobal()
{
    math::dse3 M = math::dse3::Zero();

    // TODO: Not implemented.

    return M;
}

math::dse3 Skeleton::getMomentumCOM()
{
    math::dse3 M = math::dse3::Zero();

    // TODO: Not implemented.

    return M;
}

Eigen::VectorXd Skeleton::getConfig(std::vector<int> _id)
{
    Eigen::VectorXd dofs(_id.size());

    for(unsigned int i = 0; i < _id.size(); i++)
        dofs[i] = mGenCoords[_id[i]]->get_q();

    return dofs;
}

void Skeleton::setConfig(std::vector<int> _id, Eigen::VectorXd _vals,
                         bool _calcTrans, bool _calcDeriv)
{
    for( unsigned int i = 0; i < _id.size(); i++ )
        mGenCoords[_id[i]]->set_q(_vals(i));

    // TODO: Only do the necessary updates
    if (_calcTrans)
        for (int i = 0; i < getNumBodyNodes(); i++)
            mBodyNodes.at(i)->updateTransformation();

    if (_calcDeriv)
    {
        for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
             itrJoint != mJoints.end(); ++itrJoint)
            (*itrJoint)->updateKinematics(true, false);

        for (int i = 0; i < getNumBodyNodes(); i++)
            mBodyNodes.at(i)->updateVelocity();
    }
}

} // namespace dynamics
} // namespace dart