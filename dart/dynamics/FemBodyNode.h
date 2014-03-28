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

#ifndef FEM_DYNAMICS_FEMBODYNODE_H_
#define FEM_DYNAMICS_FEMBODYNODE_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <dart/dynamics/BodyNode.h>
#include <dart/simulation/FemSim.h>

namespace dart {
namespace dynamics {

class FEMPoint;
class FEMMeshShape;

/// \brief
///
/// We assume that only one soft flesh (mesh) is contained in soft body node
class FEMBodyNode : public BodyNode
{
public:
  friend class FEMSkeleton;

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief
  explicit FEMBodyNode(const std::string& _name = "Unnamed FEMBodyNode");

  /// \brief
  virtual ~FEMBodyNode();


  /// \brief Get mass.
  double getMass() const;

  /// \brief
  void removeAllPointMasses();

  /// \brief
  void addPointMass(FEMPoint* _pointMass);

  /// \brief
  int getNumPointMasses() const;

  /// \brief
  FEMPoint* getPointMass(int _idx) const;

    /// \brief
    void addFace(const Eigen::Vector3i& _face);
    
    /// \brief
    void addTetra(int, int, int, int);
    
    /// \brief
    const Eigen::Vector3i& getFace(int _idx) const;
    
    /// \brief
    int getNumFaces();
    
    /// \brief
    void setFEMSim(dart::simulation::FemSimulation * _femsim);
    
    /// \brief
    void postAddingTetra();
    
    dart::simulation::FemSimulation * getFEMSim();
    
    /// \brief
    void initFEM();
    
protected:
  //--------------------------------------------------------------------------
  // Sub-functions for Recursive Kinematics Algorithms
  //--------------------------------------------------------------------------
  // Documentation inherited.
  virtual void init(Skeleton* _skeleton, int _skeletonIndex);

  // Documentation inherited.
  virtual void aggregateGenCoords(std::vector<GenCoord*>* _genCoords);

  // Documentation inherited.
  virtual void aggregatePointMassGenCoords(std::vector<GenCoord*>* _genCoords);

    // Documentation inherited.
    virtual void updateBodyForce(const Eigen::Vector3d& _gravity,
                                 bool _withExternalForces = false);
    
    // Documentation inherited.
    virtual void updateBiasForce(double _timeStep, const Eigen::Vector3d& _gravity);
    
  // Documentation inherited.
  virtual void update_ddq();

  // Documentation inherited.
  virtual void clearExternalForces();

  //--------------------------------------------------------------------------
  // Rendering
  //--------------------------------------------------------------------------
  /// \brief Render the entire subtree rooted at this body node.
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true, int _depth = 0) const;

protected:
  /// \brief List of point masses composing deformable mesh.
  std::vector<FEMPoint*> mPointMasses;

  // TODO(JS): Let's remove this because this is rendering part
  /// \brief Tri-mesh indexes for rendering.
  std::vector<Eigen::Vector3i> mFaces;

  /// \brief Soft mesh shape for visualization.
  FEMMeshShape* mSoftVisualShape;

  /// \brief Soft mesh shape for collision.
  FEMMeshShape* mSoftCollShape;

    /// \brief fem simulator
    dart::simulation::FemSimulation * femsim;
    
    
};

class FEMBodyNodeHelper
{
public:
  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setSingle(
      FEMBodyNode*          _softBodyNode,
      double                 _totalMass);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setBox(
      FEMBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransfom,
      double                   _totalMass);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setBox(
      FEMBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransfom,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setSinglePointMass(
      FEMBodyNode*          _softBodyNode,
      double                 _totalMass);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setEllipsoid(
      FEMBodyNode*          _softBodyNode,
      const Eigen::Vector3d& _size,
      int                    _nSlices,
      int                    _nStacks,
      double                 _totalMass);
    
};

}  // namespace dynamics
}  // namespace dart

#endif  // FEM_DYNAMICS_FEMBODYNODE_H_
