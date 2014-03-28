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


#include <iostream>
#include <string>
#include <vector>
#include <time.h>

#include "dart/simulation/FemSim.h"

namespace dart {
    namespace simulation {
        //--------------------------------------------------------------------------
        // Constructor and Destructor
        //--------------------------------------------------------------------------
        /// \brief Constructor.
        FemSimulation::FemSimulation() {
            _mass_damping = 0;
            _stiffness_damping = 0;
        }
        
        /// \brief Destructor.
        FemSimulation::~FemSimulation() {
            
        }
        
        void FemSimulation::setParameters(float ym, float pr, float md, float sd) {
            _young_mod = ym;
            _poisson_rat = pr;
            _mass_damping = md;
            _stiffness_damping = sd;
        }
        
        //--------------------------------------------------------------------------
        // Simulation
        //--------------------------------------------------------------------------
        /// \brief Update tetra information
        void FemSimulation::updateTetraState() {
            for (int i = 0; i < mTetras.size(); i ++) {
                mTetras[i]->updateStrain();
                mTetras[i]->updateStress();
            }
        }
        
        void FemSimulation::initK() {
            
            for (int i = 0; i < mTetras.size(); i ++) {
                mTetras[i]->initK();
            }
            //std::cout<<_K<<std::endl;
            //Eigen::VectorXd tt(mPoints.size()*3);
            //for (int i = 0;i < mPoints.size()*3;i ++) tt(i) = 0.5;
            //tt << 1,1,1, 1,1,1, 1,1,1, 1,1,1;
            //std::cout << tt << std::endl<<std::endl;
            //std::cout << _K*tt<<std::endl<<std::endl;
        }
        
        /// \brief update forces on points
        void FemSimulation::updateForce() {
            /*for (int i = 0; i < mTetras.size(); i ++) {
                mTetras[i]->updateTetraForce();
            }*/
            
            Eigen::VectorXd newforce(mPoints.size()*3);
            Eigen::VectorXd posdif(mPoints.size()*3);
            
            for (int i = 0; i < mPoints.size(); i ++) {
            //    posdif.segment(i*3, 3) = mPoints[i]->get_q()-mPoints[i]->getRestingPosition();
            }
            
           // newforce = _K * posdif;
            
            for (int i = 0; i < mPoints.size(); i ++) {
                //std::cout << newforce.segment(i*3,3) << std::endl << std::endl;
                //mPoints[i]->addExtForce(newforce.segment(i*3,3), false);
            }
        }
        
        /// \brief
        void FemSimulation::setPoints(std::vector<dynamics::FEMPoint* > mPts) {
            mPoints = mPts;
            for (int i = 0; i < mPoints.size(); i ++) {
                _nconstrainted_points_before.push_back(0);
            }
        }
        
        /// \brief  add new tetrahedron
        void FemSimulation::addTetra(int ind1, int ind2, int ind3, int ind4) {
            mTetras.push_back(new dynamics::FEM_Tetra(mPoints[ind1], mPoints[ind2], mPoints[ind3], mPoints[ind4], ind1, ind2, ind3, ind4));
            mTetras[mTetras.size()-1]->setYoungMod(_young_mod);
            mTetras[mTetras.size()-1]->setPoissonRat(_poisson_rat);
            mTetras[mTetras.size()-1]->buildTet();
        }
        
        void FemSimulation::postAddingTetra() {
            /*for (int i = 0; i < mPoints.size(); i ++) {
                mPoints[i]->preCompute();
            }*/
            initK();
            
            K.resize(mPoints.size()*3, mPoints.size()*3);
            Kcomplete.resize(mPoints.size()*3, mPoints.size()*3);
            M.resize(mPoints.size()*3, mPoints.size()*3);
            // aggregate mass matrix M
            for (int i = 0; i < mPoints.size()*3; i ++) {
                M.coeffRef(i, i) = mPoints[i/3]->getMass();
            }
            
            // std::cout << mPoints.size() << " " << mTetras.size() << std::endl;
        }
        
        
        void FemSimulation::step(float dt) {
            //updateTetraState();
            //updateForce();
            
            // explicit euler
            /*for (int i = 0; i < mPoints.size(); i ++) {
                Eigen::VectorXd tempq = mPoints[i]->get_q();
                Eigen::VectorXd tempdq = mPoints[i]->get_dq();
                Eigen::VectorXd tempddq = mPoints[i]->get_ddq();
                
                tempq += dt*tempdq;
                tempdq += dt*tempddq;
                
                mPoints[i]->set_q(tempq);
                mPoints[i]->set_dq(tempdq);
            }*/
            
            // implicit euler
            int dim = mPoints.size()-mContraintPoints.size();
            Eigen::SparseMatrix<double> A(dim*3, dim*3);
            K.resize(dim*3, dim*3);
            M.resize(dim*3, dim*3);
            static Eigen::VectorXd newforce(mPoints.size()*3);
            static Eigen::VectorXd curpos(mPoints.size()*3);
            static Eigen::VectorXd f0(mPoints.size()*3);
            Eigen::VectorXd b(mPoints.size()*3);
            Eigen::VectorXd x(mPoints.size()*3);
            
            K.setZero();
            A.setZero();
            C.setZero();
            newforce.setZero();
            b.setZero();
            f0.setZero();
            
            clock_t t, e;
            t = clock();
            std::vector<Eigen::Triplet<double> > tripletListK, tripletListKC, tripletListM;
            tripletListK.reserve(4*mPoints.size());
            tripletListKC.reserve(4*mPoints.size());
            tripletListM.reserve(3*mPoints.size());
            // aggregate quantities
            aggregateM(tripletListM, _nconstrainted_points_before);
            for (int i = 0; i < mTetras.size(); i ++) {
                mTetras[i]->updateRotationMatrix();
                mTetras[i]->aggregateK(tripletListK, _nconstrainted_points_before);
                mTetras[i]->aggregateK(tripletListKC);
                // mTetras[i]->aggregateK(K);
                mTetras[i]->aggregateF0(f0);
            }
            //std::cout << "size:" << tripletListK.size() << std::endl;
            //std::cout << "sizeM:" << tripletListM.size() << std::endl;
            //std::cout << "size2:" << K.rows() << " " << K.cols() << std::endl;
            //t = clock();
            K.setFromTriplets(tripletListK.begin(), tripletListK.end());
            Kcomplete.setFromTriplets(tripletListKC.begin(), tripletListKC.end());
            M.setFromTriplets(tripletListM.begin(), tripletListM.end());
            //e = clock();
            //std::cout << "nonzeros:" << K.nonZeros() << std::endl;
            //std::cout << "He Mengmeng\n";
            //std::cout << mTetras.size() << std::endl;
            //std::cout << "time" << e-t << std::endl;
            
            
            //std::cout << f0 << std::endl << std::endl;
            //std::cout<<"K:"<<K<<std::endl<<std::endl;
            //std::cout<<"M:" << M << std::endl << std::endl;
            //Eigen::VectorXd tt(mPoints.size()*3);
            //for (int i = 0;i < mPoints.size()*3;i ++) tt(i) = 0.5;
            //tt << 1,1,1, 1,1,1, 1,1,1, 1,1,1;
            //std::cout << tt << std::endl<<std::endl;
            //std::cout << K*tt<<std::endl<<std::endl;
            
            
            // compute for A
            A = M - dt*dt*K + dt*(_mass_damping*M + _stiffness_damping*K);
                        //std::cout<< A.isCompressed() << std::endl;
            // compute for b
            for (int i = 0; i < mPoints.size(); i ++) {
                curpos.segment(i*3, 3) = mPoints[i]->get_q();
                //restpos.segment(i*3, 3) = mPoints[i]->getRestingPosition();
            }
            
            newforce = Kcomplete*curpos - f0;
            //std::cout << K*curpos << std::endl << f0 << std::endl << std::endl;
            for (int i = 0; i < mPoints.size(); i ++) {
                newforce.segment(i*3, 3) += mPoints[i]->getExtForce();
            }
            
            for (int i = 0; i < mPoints.size(); i ++) {
                b.segment(i*3,3) = mPoints[i]->getMass()*mPoints[i]->get_dq();
            }
            b += dt*newforce;
            
            removeConstraints(b);
            
            //std::cout << A.rows() << " " << A.cols() << " " << b.rows() << std::endl;
            
            // solve Ax = b
            Eigen::ConjugateGradient<Eigen::SparseMatrix<double> > cg;
            cg.compute(A);
            x = cg.solve(b);
            addConstraintsBack(x);
            for (int i = 0; i < mPoints.size(); i ++) {
                mPoints[i]->set_dq(x.segment(i*3,3));
                mPoints[i]->set_q(curpos.segment(i*3,3) + dt*x.segment(i*3,3));
                mPoints[i]->clearExtForce();
            }
        }
        
        void FemSimulation::addConstraintPoint(dynamics::FEMPoint* p) {
            mContraintPoints.push_back(p);
        }
        
        // update the _nconstrainted_points_before array
        void FemSimulation::updateConstraintCountArray() {
            int nbefore = 0;
            for (int i = 0; i < mPoints.size(); i ++) {
                _nconstrainted_points_before[i] = nbefore;
                if (mPoints[i]->isImmobile()) {
                    nbefore ++;
                }
            }
        }
        
        void FemSimulation::addControlledpoint(dynamics::FEMPoint* p) {
            mControlledPoints.push_back(p);
        }
        
        void FemSimulation::stretch(Eigen::Vector3d dif) {
            //static Eigen::Vector3d STRETCHFORCE = Eigen::Vector3d::Zero();
            //STRETCHFORCE += dif;
                for (int i = 0; i < mControlledPoints.size(); i ++) {
                    //mControlledPoints[i]->addExtForce(STRETCHFORCE);
                    //std::cout << mContraintPoints[i]->getExtForce() << std::endl;
                    Eigen::Vector3d newpos = mControlledPoints[i]->get_q() + dif;
                    mControlledPoints[i]->set_q(newpos);
                }
        }
        
        void FemSimulation::rotate(float ang) {
            float radian = ang/180*3.1415926;
            
            for (int i = 0; i < mControlledPoints.size(); i ++) {
                Eigen::Vector3d pos = mControlledPoints[i]->get_q();
                Eigen::Vector3d newpos;
                newpos.x() = cos(radian)*pos.x() + sin(radian)*pos.y();
                newpos.y() = -sin(radian)*pos.x() + cos(radian)*pos.y();
                newpos.z() = pos.z();
                    
                mControlledPoints[i]->set_q(newpos);
            }
        }
        
        void FemSimulation::free() {
            for (int i = 0; i < mControlledPoints.size(); i ++) {
                mControlledPoints[i]->setImmobile(false);
            }
            
            //std::cout << mContraintPoints.size() << std::endl;
            
            std::vector<dynamics::FEMPoint* >::iterator iter;
            for (int i = 0; i < mControlledPoints.size(); i ++) {
                //std::cout << mContraintPoints.size() << std::endl;
                for (iter = mContraintPoints.begin(); iter != mContraintPoints.end() ; iter ++) {
                    if (*iter == mControlledPoints[i]) {
                        mContraintPoints.erase(iter);
                        break;
                    }
                }
            }
            updateConstraintCountArray();
        }
        
        void FemSimulation::removeConstraints(Eigen::VectorXd& v) {
            int dim = v.rows() - mContraintPoints.size()*3;
            
            Eigen::VectorXd newv(dim);
            int curp = 0;
            for (int i = 0; i < mPoints.size(); i ++) {
                if (!mPoints[i]->isImmobile()) {
                    newv.segment(curp*3, 3) = v.segment(i*3, 3);
                    curp ++;
                }
            }
            v.resize(dim);
            v = newv;
        }
        
        void FemSimulation::addConstraintsBack(Eigen::VectorXd& v) {
            int dim = v.rows() + mContraintPoints.size()*3;
            
            Eigen::VectorXd newv(dim);
            int curp = 0;
            for (int i = 0; i < mPoints.size(); i ++) {
                if (!mPoints[i]->isImmobile()) {
                    newv.segment(i*3, 3) = v.segment(curp*3, 3);
                    curp ++;
                }
                else {
                    newv.segment(i*3, 3) = Eigen::Vector3d::Zero();
                }
            }
            v.resize(dim);
            v = newv;
        }
        
        void FemSimulation::aggregateM(std::vector<Eigen::Triplet<double> > &tripletList, std::vector<int> num) {
            for (int i = 0; i < mPoints.size(); i ++) {
                if (mPoints[i]->isImmobile()) {
                    continue;
                }
                for (int j = 0; j < 3; j ++) {
                    //std::cout << i << std::endl;
                    tripletList.push_back(Eigen::Triplet<double>((i-num[i])*3+j, (i-num[i])*3+j, mPoints[i]->getMass()));
                }
            }
            
        }
        
    }  // namespace simulation
}  // namespace dart
