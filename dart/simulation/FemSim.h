/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifndef DART_FEM_SIMULATION_H
#define DART_FEM_SIMULATION_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <dart/dynamics/FemTetra.h>


namespace dart {
    
    
namespace simulation {

/// \class FemSimulation
/// \brief
class FemSimulation {
public:
  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------
  /// \brief Constructor.
  FemSimulation();

  /// \brief Destructor.
  virtual ~FemSimulation();

    void setParameters(float ym, float pr, float md = 0, float sd = 0);
    
  //--------------------------------------------------------------------------
  // Simulation
  //--------------------------------------------------------------------------
  /// \brief Update tetra information
  void updateTetraState();
    
    /// \brief update forces on points
    void updateForce();
    
    void initK();
    
    /// \brief
    void setPoints(std::vector<dynamics::FEMPoint* > mPts);
    
    void addPoint(dynamics::FEMPoint* pt);
    
    /// \brief  add new tetrahedron
    void addTetra(int ind1, int ind2, int ind3, int ind4);

    void postAddingTetra();
    
    std::vector<dynamics::FEM_Tetra* > getTetras() {return mTetras;}
    
    void step(float dt);
    
    void addConstraintPoint(dynamics::FEMPoint*);
    
    void addControlledpoint(dynamics::FEMPoint*);
    
    void stretch(Eigen::Vector3d);
    void rotate(float);
    void free();
    
    void removeConstraints(Eigen::VectorXd&);
    
    void addConstraintsBack(Eigen::VectorXd&);
    
    // update the _nconstrainted_points_before array
    void updateConstraintCountArray();
   
    void aggregateM(std::vector<Eigen::Triplet<double> > &tripletList, std::vector<int> num);
protected:
    std::vector<dynamics::FEMPoint* > mPoints;
    
    std::vector<dynamics::FEM_Tetra* > mTetras;
    
    Eigen::SparseMatrix<double> K;  // stiffness matrix
    
    Eigen::SparseMatrix<double> Kcomplete;  //entire stiffness matrix
    
    Eigen::SparseMatrix<double> M;  // mass matrix
    
    Eigen::SparseMatrix<double> C;  // damping matrix
    
    float _young_mod;
    
    float _poisson_rat;
    
    float _mass_damping;
    
    float _stiffness_damping;
    
    std::vector<dynamics::FEMPoint*> mContraintPoints;
    
    std::vector<dynamics::FEMPoint*> mControlledPoints;
    
    std::vector<int> _nconstrainted_points_before;
private:
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace simulation
}  // namespace dart

#endif  // DART_FEM_SIMULATION_H
