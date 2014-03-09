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

#include "dart/dynamics/FemTetra.h"

using namespace Eigen;

namespace dart {
    namespace dynamics {
        FEM_Tetra::FEM_Tetra() {}
        
        FEM_Tetra::FEM_Tetra(FEMPoint* pt1, FEMPoint* pt2, FEMPoint* pt3, FEMPoint* pt4, int p1, int p2, int p3, int p4) {
            setPointMass(pt1, pt2, pt3, pt4, p1, p2, p3, p4);
        }
        
        /// \brief Default destructor
        FEM_Tetra::~FEM_Tetra() {}
        
        void FEM_Tetra::setYoungMod(float ym) {
            young_mod = ym;
        }
        
        /// \brief
        void FEM_Tetra::setPoissonRat(float pr) {
            poiss_rat = pr;
        }
        
        ///  Compute strain for this tetra
        void FEM_Tetra::updateStrain() {
            Eigen::Matrix3d temp;
            temp << mPoints[1]->getPosition()-mPoints[0]->getPosition(), mPoints[2]->getPosition()-mPoints[0]->getPosition(), mPoints[3]->getPosition()-mPoints[0]->getPosition();
            
            //std::cout<<_strain<<std::endl<<rest_pos_inv<<std::endl<<std::endl;
            _strain = temp * rest_pos_inv - Eigen::Matrix3d::Identity();
            
            temp = _strain.transpose();
            
            _strain = 0.5 * (_strain + temp);
            
            
        }
        
        ///  Compute stress for this tetra
        void FEM_Tetra::updateStress() {
            float mult = young_mod/(1+poiss_rat)/(1-2*poiss_rat);
            Eigen::Matrix3d temp;
            
            _stress(0,0) = _strain(0,0)*(1-poiss_rat)+_strain(1,1)*poiss_rat+_strain(2,2)*poiss_rat;
            _stress(1,1) = _strain(0,0)*poiss_rat+_strain(1,1)*(1-poiss_rat)+_strain(2,2)*poiss_rat;
            _stress(2,2) = _strain(0,0)*poiss_rat+_strain(1,1)*poiss_rat+_strain(2,2)*(1-poiss_rat);
            _stress(1,0) = _stress(0,1) = (1-2*poiss_rat)*_strain(1,0);
            _stress(1,2) = _stress(2,1) = (1-2*poiss_rat)*_strain(1,2);
            _stress(2,0) = _stress(0,2) = (1-2*poiss_rat)*_strain(2,0);
            
            temp = _stress;
            
            _stress = mult * temp;
        }
        
        ///  Compute stress force and distribute to the four vertices
        void FEM_Tetra::updateTetraForce() {
            for (int i = 0 ;i < 4; i ++ ) {
                
                ///  f face = stress * normal * face_area
                Eigen::Vector3d fface = -_stress * ((mPoints[(i+1)%4]->getPosition()-mPoints[i%4]->getPosition()).cross((mPoints[(i+2)%4]->getPosition()-mPoints[i%4]->getPosition())));
                if (i%2 != 0) {
                    fface = -fface;
                }
                
                //std::cout << fface << "\n\n";
                
                mPoints[i%4]->addExtForce(1/3.0*fface, false);
                mPoints[(i+1)%4]->addExtForce(1/3.0*fface, false);
                mPoints[(i+2)%4]->addExtForce(1/3.0*fface, false);
                
            }
        }
        
        ///
        void FEM_Tetra::setPointMass(FEMPoint* pt1, FEMPoint* pt2, FEMPoint* pt3, FEMPoint* pt4,
                                     int p1, int p2, int p3, int p4) {
            /// check the sequence
            
            if (((pt2->getRestingPosition()-pt1->getRestingPosition()).cross(pt3->getRestingPosition()-pt1->getRestingPosition())).dot(pt4->getRestingPosition()-pt1->getRestingPosition()) > 0)
            {
                mPoints[0] = pt1; mPoints[1] = pt3; mPoints[2] = pt2; mPoints[3] = pt4;
                mPointIndexes[0] = p1; mPointIndexes[1] = p3; mPointIndexes[2] = p2; mPointIndexes[3] = p4;
            }
            else {
                mPoints[0] = pt1; mPoints[1] = pt2; mPoints[2] = pt3; mPoints[3] = pt4;
                mPointIndexes[0] = p1; mPointIndexes[1] = p2; mPointIndexes[2] = p3; mPointIndexes[3] = p4;
            }
        }
        
        ///  precompute
        void FEM_Tetra::buildTet() {
            Eigen::Matrix3d temp;
            temp << mPoints[1]->getRestingPosition()-mPoints[0]->getRestingPosition(), mPoints[2]->getRestingPosition()-mPoints[0]->getRestingPosition(), mPoints[3]->getRestingPosition()-mPoints[0]->getRestingPosition();
            
            //std::cout<<temp<<std::endl;
            
            rest_pos_inv = temp.inverse();
            
            /*// add connected point to points
            for (int i = 0; i < 4; i ++) {
                for (int j = 3; j >= 1; j --) {
                    mPoints[i]->addConnectedPoint(mPoints[(i+j)%4]);
                }
            }*/
            
            //std::cout<<rest_pos_inv<<std::endl;
        }
        
        void FEM_Tetra::initK() {
            // compute for submatrix Kij relating v_i and v_j
            Eigen::Matrix3d mid1, mid2, ele1, ele2, ele3, ele4;
            
            Eigen::Matrix3d temp;
            temp << mPoints[1]->getRestingPosition()-mPoints[0]->getRestingPosition(), mPoints[2]->getRestingPosition()-mPoints[0]->getRestingPosition(), mPoints[3]->getRestingPosition()-mPoints[0]->getRestingPosition();
            
            Eigen::Vector3d ys[4];
            ys[1] = rest_pos_inv.row(0);
            ys[2] = rest_pos_inv.row(1);
            ys[3] = rest_pos_inv.row(2);
            ys[0] = -ys[1]-ys[2]-ys[3];
            
            float a = temp.determinant() * young_mod * (1-poiss_rat)/(1+poiss_rat)/(1-2*poiss_rat);
            float b = temp.determinant() * young_mod * poiss_rat/(1+poiss_rat)/(1-2*poiss_rat);
            float c = temp.determinant() * young_mod * (1-2*poiss_rat)/(1+poiss_rat)/(1-2*poiss_rat);
            
            mid1 << a, b, b,
            b, a, b,
            b, b, a;
            
            mid2 << c, 0, 0,
            0, c, 0,
            0, 0, c;
            
            for (int i = 0; i < 4; i ++) {
                for (int j = i ; j < 4; j ++) {
                    ele1 << ys[i].x(),     0,      0,
                    0,          ys[i].y(), 0,
                    0,          0,      ys[i].z();
                    
                    ele2 << ys[j].x(),     0,      0,
                    0,          ys[j].y(), 0,
                    0,          0,      ys[j].z();
                    
                    ele3 << ys[i].y(), 0, ys[i].z(),
                    ys[i].x(), ys[i].z(), 0,
                    0, ys[i].y(), ys[i].x();
                    
                    ele4 << ys[j].y(), ys[j].x(), 0,
                    0, ys[j].z(), ys[j].y(),
                    ys[j].z(), 0, ys[j].x();
                    
                    _K[i*(9-i)/2+j-i] = ele1*mid1*ele2+ele3*mid2*ele4;
                }
            }
            
            for (int i = 0; i < 4; i ++) {
                _f0[i].setZero();
            }
            for (int j = 0; j < 4; j ++) {
                //_f0[i].setZero();
                for (int i = 0; i < 4; i ++) {
                    if (j < i)
                        _f0[i] += _K[j*(9-j)/2+i-j].transpose() * mPoints[j]->getRestingPosition();
                    else
                        _f0[i] += _K[i*(9-i)/2+j-i] * mPoints[j]->getRestingPosition();

                }
            }
        }
        
        void FEM_Tetra::aggregateF0(Eigen::VectorXd& f0) {
            for (int i = 0; i < 4; i ++) {
                f0.segment(mPointIndexes[i]*3, 3) += _R*_f0[i];
            }
        }
        
        void FEM_Tetra::aggregateK(Eigen::SparseMatrix<double>& K) {
            // construct the entire K
            static Eigen::Matrix3d tempmat;
            for (int i = 0; i < 4; i ++) {
                for (int j = i; j < 4; j ++) {
                    tempmat = _R*_K[i*(9-i)/2+j-i]*_R.transpose();
                    for (int k = 0; k < 3; k ++) {
                        for (int l = 0; l < 3; l ++) {
                            K.coeffRef(mPointIndexes[i]*3+k, mPointIndexes[j]*3+l) += tempmat(k,l);
                            if (i != j)
                                K.coeffRef(mPointIndexes[j]*3+l, mPointIndexes[i]*3+k) += tempmat(k,l);
                        }
                    }
                }
            }
        }
    
        void FEM_Tetra::updateRotationMatrix() {
            Eigen::Matrix3d p, A;
            p << mPoints[1]->get_q()-mPoints[0]->get_q(), mPoints[2]->get_q()-mPoints[0]->get_q(), mPoints[3]->get_q()-mPoints[0]->get_q();
            
            A = p * rest_pos_inv;
            
            Eigen::Vector3d r0, r1, r2;
            
            r0 = A.col(0);
            r0.normalize();
            r1 = A.col(1) - r0.dot(A.col(1))*r0;
            r1.normalize();
            r2 = r0.cross(r1);
            r2.normalize();
            
            _R << r0, r1, r2;
            
            /*Eigen::Matrix3d N, N2;
            
            Vector3d v1, v2, v3;
            
            v1 = mPoints[1]->getRestingPosition() - mPoints[0]->getRestingPosition();
            v2 = mPoints[2]->getRestingPosition() - mPoints[0]->getRestingPosition();
            v3 = mPoints[3]->getRestingPosition() - mPoints[0]->getRestingPosition();
            v1.normalize(); v2.normalize(); v3.normalize();
            Eigen::Vector3d n1, n2, n3;
            n1 = (v1+v2+v3); n1.normalize();
            n2 = v1.cross(n1); n2.normalize();
            n3 = n1.cross(n2); n3.normalize();
            
            N << n1, n2, n3;
            
            v1 = mPoints[1]->get_q() - mPoints[0]->get_q();
            v2 = mPoints[2]->get_q() - mPoints[0]->get_q();
            v3 = mPoints[3]->get_q() - mPoints[0]->get_q();
            v1.normalize(); v2.normalize(); v3.normalize();
            n1 = (v1+v2+v3); n1.normalize();
            n2 = v1.cross(n1); n2.normalize();
            n3 = n1.cross(n2); n3.normalize();
            
            N2 << n1, n2, n3;
            
            _R = N2*N.transpose();*/
        }
        
    }   // namespace dynamics
}   //namespace dart

