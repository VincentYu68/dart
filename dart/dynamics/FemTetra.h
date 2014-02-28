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

#ifndef FEM_DYNAMICS_TETRA_H
#define FEM_DYNAMICS_TETRA_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <dart/dynamics/FemPoint.h>

namespace dart {
    namespace dynamics {

        

/// \brief
class FEM_Tetra
{
public:
  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief Default constructor
    FEM_Tetra();

    FEM_Tetra(FEMPoint* pt1, FEMPoint* pt2, FEMPoint* pt3, FEMPoint* pt4, int p1, int p2, int p3, int p4);
    
  /// \brief Default destructor
  virtual ~FEM_Tetra();
                     /// \brief
    void setYoungMod(float ym);
    
    /// \brief
    void setPoissonRat(float pr);
    
                     /// \brief
                     Eigen::Matrix3d get_Strain() {return _strain;}
                     
                     /// \brief
                     Eigen::Matrix3d get_Stress() {return _stress;}
                     
                     /// \brief Compute strain for this tetra
                     void updateStrain();
                     
                     /// \brief Compute stress for this tetra
                     void updateStress();
                     
                     /// \brief Compute stress force and distribute to the four vertices
                     void updateTetraForce();
                     
                     /// \brief
                     void setPointMass(FEMPoint* pt1, FEMPoint* pt2, FEMPoint* pt3, FEMPoint* pt4, int p1, int p2, int p3, int p4);
                     
                     /// \brief precompute
                     void buildTet();
    
    FEMPoint *getPoint(int i) {return mPoints[i];}
    
    void initK(Eigen::SparseMatrix<double>&);
protected:
  
                     
protected:
  /// \brief
  FEMPoint* mPoints[4];

    int mPointIndexes[4];
    
private:
  /// \brief
    float young_mod;           // young's modulous
    float poiss_rat;           // poisson ratio
    Eigen::Matrix3d _strain;   //strain tensor
    Eigen::Matrix3d _stress;   //stress tensor
    
    /// \brief
    Eigen::Matrix3d rest_pos_inv;
                     
                     
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}  // namespace dynamics
}  // namespace dart

#endif  // FEM_DYNAMICS_TETRA_H
