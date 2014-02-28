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

#ifndef FEM_DYNAMICS_POINT_H_
#define FEM_DYNAMICS_POINT_H_

#include <vector>
#include <Eigen/Dense>
#include <dart/dynamics/GenCoordSystem.h>

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class EllipsoidShape;
class FEMBodyNode;

/// \brief
class FEMPoint : public GenCoordSystem
{
public:
    friend class FEMBodyNode;
  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief Default constructor
  explicit FEMPoint(FEMBodyNode* _femBodyNode);

  /// \brief Default destructor
  virtual ~FEMPoint();

  /// \brief
  void setMass(double _mass);

  /// \brief
  double getMass() const;

  /// \brief Set whether this point mass is colliding with others.
  /// \param[in] True if this point mass is colliding.
  void setColliding(bool _isColliding);

  /// \brief Get whether this point mass is colliding with others.
  /// \return True if this point mass is colliding.
  bool isColliding();

  /// \brief Add linear Cartesian force to this node.
  /// \param[in] _force External force.
  /// \param[in] _isForceLocal True if _force's reference frame is of the parent
  ///                          soft body node. False if _force's reference frame
  ///                          is of the world.
    void setImmobile(bool i);
    
    bool isImmobile();
    
    void addExtForce(const Eigen::Vector3d& _force, bool _isForceLocal = false);
  /// \brief
  void clearExtForce();

    Eigen::Vector3d getExtForce();
    
  /// \brief
  void setRestingPosition(const Eigen::Vector3d& _p);

  /// \brief
  const Eigen::Vector3d& getRestingPosition() const;

  /// \brief
  const Eigen::Vector3d getPosition() const;

    FEMBodyNode* getParentFEMBodyNode() const;
    
    int getNumDependentGenCoords() const;
    
    int getDependentGenCoord(int _arrayIndex) const;
    
    void setGravity(const Eigen::Vector3d& _g);

protected:
  /// \brief
  void init();
    
  /// \brief
  void update_ddq();

 

  //---------------------------- Rendering -------------------------------------
  /// \brief
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true) const;

protected:
  /// \brief
  GenCoord mCoordinate[3];

  /// \brief Mass.
  double mMass;

  /// \brief Current position viewed in world frame.
  Eigen::Vector3d mX;

  /// \brief Resting postion viewed in world frame.
  Eigen::Vector3d mX0;

  /// \brief Current velocity viewed in world frame.
  Eigen::Vector3d mV;
    
  /// \brief
  Eigen::Vector3d mGravity;

  /// \brief
  FEMBodyNode* mParentFEMBodyNode;

  /// \brief External force.
  Eigen::Vector3d mFext;
    
    /// \brief total force
    Eigen::Vector3d mForce;

  /// \brief Whether the node is currently in collision with another node.
  bool mIsColliding;
    
    bool immobile;
    
    /// \brief A increasingly sorted list of dependent dof indices.
    std::vector<int> mDependentGenCoordIndices;

private:
  EllipsoidShape* mShape;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // FEM_DYNAMICS_POINT_H_
