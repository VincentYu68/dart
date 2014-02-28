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

#include "dart/dynamics/FemBodyNode.h"

#include <string>
#include <vector>

#include <dart/common/Console.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/Shape.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/renderer/LoadOpengl.h>
#include <dart/renderer/RenderInterface.h>
#include "dart/dynamics/FemPoint.h"
#include "dart/dynamics/FemMeshShape.h"

namespace dart {
namespace dynamics {

FEMBodyNode::FEMBodyNode(const std::string& _name)
  : BodyNode(_name),
    mSoftVisualShape(NULL),
    mSoftCollShape(NULL)
{
}

FEMBodyNode::~FEMBodyNode()
{
  for (int i = 0; i < mPointMasses.size(); ++i)
    delete mPointMasses[i];

  delete femsim;
}

int FEMBodyNode::getNumPointMasses() const
{
  return mPointMasses.size();
}

FEMPoint* FEMBodyNode::getPointMass(int _idx) const
{
  assert(0 <= _idx && _idx < mPointMasses.size());
  return mPointMasses[_idx];
}

void FEMBodyNode::init(Skeleton* _skeleton, int _skeletonIndex)
{
  BodyNode::init(_skeleton, _skeletonIndex);

  for (int i = 0; i < mPointMasses.size(); ++i)
    mPointMasses[i]->init();

//  //----------------------------------------------------------------------------
//  // Visualization shape
//  //----------------------------------------------------------------------------
//  assert(mSoftVisualShape == NULL);
//  mSoftVisualShape = new SoftMeshShape(this);
//  BodyNode::addVisualizationShape(mSoftVisualShape);

//  //----------------------------------------------------------------------------
//  // Collision shape
//  //----------------------------------------------------------------------------
//  assert(mSoftCollShape == NULL);
//  mSoftCollShape = new SoftMeshShape(this);
//  BodyNode::addCollisionShape(mSoftCollShape);
}
    void FEMBodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords)
    {
        BodyNode::aggregateGenCoords(_genCoords);
        aggregatePointMassGenCoords(_genCoords);
    }
    
    void FEMBodyNode::aggregatePointMassGenCoords(
                                               std::vector<GenCoord*>* _genCoords)
    {
        for (int i = 0; i < getNumPointMasses(); ++i)
        {
            FEMPoint* pointMass = getPointMass(i);
            for (int j = 0; j < pointMass->getNumGenCoords(); ++j)
            {
                GenCoord* genCoord = pointMass->getGenCoord(j);
                genCoord->setSkeletonIndex(_genCoords->size());
                _genCoords->push_back(genCoord);
            }
        }
    }

double FEMBodyNode::getMass() const
{
  double totalMass = BodyNode::getMass();

  for (int i = 0; i < mPointMasses.size(); ++i)
    totalMass += mPointMasses.at(i)->getMass();

  return totalMass;
}

void FEMBodyNode::removeAllPointMasses()
{
  mPointMasses.clear();
}

void FEMBodyNode::addPointMass(FEMPoint* _pointMass)
{
  assert(_pointMass != NULL);
  mPointMasses.push_back(_pointMass);
}

void FEMBodyNode::addFace(const Eigen::Vector3i& _face)
{
  assert(_face[0] != _face[1]);
  assert(_face[1] != _face[2]);
  assert(_face[2] != _face[0]);
  assert(0 <= _face[0] && _face[0] < mPointMasses.size());
  assert(0 <= _face[1] && _face[1] < mPointMasses.size());
  assert(0 <= _face[2] && _face[2] < mPointMasses.size());
  mFaces.push_back(_face);
}
    
    void FEMBodyNode::addTetra(int p1, int p2, int p3, int p4) {
        femsim->addTetra(p1, p2, p3, p4);
    }

const Eigen::Vector3i& FEMBodyNode::getFace(int _idx) const
{
  assert(0 <= _idx && _idx < mFaces.size());
  return mFaces[_idx];
}

int FEMBodyNode::getNumFaces()
{
  return mFaces.size();
}

    void FEMBodyNode::setFEMSim(dart::simulation::FemSimulation * _femsim) {
        femsim = _femsim;
    }

    void FEMBodyNode::postAddingTetra() {
        femsim->postAddingTetra();
    }
    
    dart::simulation::FemSimulation * FEMBodyNode::getFEMSim() {
        return femsim;
    }
    
    void FEMBodyNode::initFEM() {
        femsim->setPoints(mPointMasses);
    }
    
    void FEMBodyNode::updateBodyForce(const Eigen::Vector3d& _gravity,
                                       bool _withExternalForces)
    {
        for (int i = 0; i < mPointMasses.size(); ++i) {
            //mPointMasses.at(i)->addExtForce(_gravity);
        }
    }
    
    void FEMBodyNode::updateBiasForce(double _timeStep, const Eigen::Vector3d& _gravity) {
        
        for (int i = 0; i < mPointMasses.size(); ++i) {
            mPointMasses.at(i)->clearExtForce();
            mPointMasses.at(i)->addExtForce(_gravity*mPointMasses.at(i)->getMass());
        }
    }

void FEMBodyNode::update_ddq()
{
    
    /*Eigen::Vector3d restingPos0 = Eigen::Vector3d(0.0,     std::sqrt(6)/3.0,   0.0);
    Eigen::Vector3d restingPos1 = Eigen::Vector3d(0.0,     0.0,                -std::sqrt(3)/3.0);
    Eigen::Vector3d restingPos2 = Eigen::Vector3d(-0.5,    0.0,                std::sqrt(3)/6.0);
    Eigen::Vector3d restingPos3 = Eigen::Vector3d(0.5,     0.0,                std::sqrt(3)/6.0);
    
    mPointMasses.at(2)->set_q(restingPos0);
    mPointMasses.at(0)->set_q(restingPos1);
    mPointMasses.at(1)->set_q(restingPos2);
    mPointMasses.at(4)->set_q(restingPos3);*/
    
    femsim->updateTetraState();
    //std::cout << "st cmp fors\n";
    femsim->updateForce();
    //std::cout << "fnsh cmp fors\n";

    for (int i = 0; i < mPointMasses.size(); ++i) {
        mPointMasses.at(i)->update_ddq();
    }
}

void FEMBodyNode::clearExternalForces()
{
  BodyNode::clearExternalForces();

  for (int i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearExtForce();
}

void FEMBodyNode::draw(renderer::RenderInterface* _ri,
                        const Eigen::Vector4d& _color,
                        bool _useDefaultColor,
                        int _depth) const
{
  if (_ri == NULL)
    return;

  _ri->pushMatrix();

  // render the self geometry
  mParentJoint->applyGLTransform(_ri);

  _ri->pushName((unsigned)mID);
  // rigid body
  for (int i = 0; i < mVizShapes.size(); i++)
  {
    _ri->pushMatrix();
    //mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }

  // vertex
//  if (_showPointMasses)
  {
    for (int i = 0; i < mPointMasses.size(); ++i)
    {
      _ri->pushMatrix();
      mPointMasses[i]->draw(_ri, _color, _useDefaultColor);
      _ri->popMatrix();
    }
  }

  // edges (mesh)
  Eigen::Vector4d fleshColor = _color;
  fleshColor[3] = 0.5;
  _ri->setPenColor(fleshColor);
//  if (_showMeshs)
  {
    Eigen::Vector3d pos;
    Eigen::Vector3d pos_normalized;
      
      std::vector<dynamics::FEM_Tetra* > mtetra = femsim->getTetras();
      for (int i = 0;i < mtetra.size();i ++) {
       for (int j = 0; j < 4; j ++) {
       glEnable(GL_AUTO_NORMAL);
       glBegin(GL_TRIANGLES);
       
       pos = mtetra[i]->getPoint(j%4)->getPosition();
       pos_normalized = pos.normalized();
       glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
       glVertex3f(pos(0), pos(1), pos(2));
       pos = mtetra[i]->getPoint((j+1)%4)->getPosition();
       pos_normalized = pos.normalized();
       glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
       glVertex3f(pos(0), pos(1), pos(2));
       pos = mtetra[i]->getPoint((j+2)%4)->getPosition();
       pos_normalized = pos.normalized();
       glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
       glVertex3f(pos(0), pos(1), pos(2));
       glEnd();
       }
    }
      
    /*for (int i = 0; i < mFaces.size(); ++i)
    {
      glEnable(GL_AUTO_NORMAL);
      glBegin(GL_TRIANGLES);

      pos = mPointMasses[mFaces[i](0)]->getPosition();
      pos_normalized = pos.normalized();
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](1)]->getPosition();
      pos_normalized = pos.normalized();
     //   std::cout<<pos<<std::endl;
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      pos = mPointMasses[mFaces[i](2)]->getPosition();
      pos_normalized = pos.normalized();
     //   std::cout<<pos<<std::endl<<std::endl;
      glNormal3f(pos_normalized(0), pos_normalized(1), pos_normalized(2));
      glVertex3f(pos(0), pos(1), pos(2));
      glEnd();
    }*/
  }

  _ri->popName();

  // render the subtree
  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
  {
    getChildBodyNode(i)->draw(_ri, _color, _useDefaultColor);
  }

  _ri->popMatrix();
}



void FEMBodyNodeHelper::setBox(FEMBodyNode*            _femBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransfom,
                                double                   _totalMass)
{
  assert(_femBodyNode != NULL);
    
  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = 76;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
    restingPos[0] = Eigen::Vector3d(-1.0, -1.0, -1.0) * 0.5;
    restingPos[1] = Eigen::Vector3d(+1.0, -1.0, -1.0) * 0.5;
    restingPos[2] = Eigen::Vector3d(-1.0, +1.0, -1.0) * 0.5;
    restingPos[3] = Eigen::Vector3d(+1.0, +1.0, -1.0) * 0.5;
    restingPos[4] = Eigen::Vector3d(-1.0, -1.0, +1.0) * 0.5;
    restingPos[5] = Eigen::Vector3d(+1.0, -1.0, +1.0) * 0.5;
    restingPos[6] = Eigen::Vector3d(-1.0, +1.0, +1.0) * 0.5;
    restingPos[7] = Eigen::Vector3d(+1.0, +1.0, +1.0) * 0.5;

    for (int i = 8; i < nPointMasses; i ++) {
        restingPos[i] = restingPos[i-4] + Eigen::Vector3d(0,0,1);
    }

    
    /*restingPos[0] = Eigen::Vector3d(0.0,     std::sqrt(6)/3.0,   0.0);
     restingPos[1] = Eigen::Vector3d(0.0,     0.0,                -std::sqrt(3)/3.0);
     restingPos[2] = Eigen::Vector3d(-0.5,    0.0,                std::sqrt(3)/6.0);
     restingPos[3] = Eigen::Vector3d(0.5,     0.0,                std::sqrt(3)/6.0);*/
    
  // Point masses
  dynamics::FEMPoint* newPointMass = NULL;
  for (int i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new FEMPoint(_femBodyNode);
    newPointMass->setRestingPosition(restingPos[i]);
    newPointMass->setMass(mass);
      
      //if (i <= 3) newPointMass->setImmobile(true);
      if (i == 2 || i == 3 || i == 7 || i == 6) newPointMass->setImmobile(true);
      //if (i == 0) newPointMass->setImmobile(true);
      
    _femBodyNode->addPointMass(newPointMass);
  }
    
    _femBodyNode->initFEM();

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
    for (int i = 0; i < (nPointMasses-4)/4 ; i ++) {
        // -- +Z
        _femBodyNode->addFace(Eigen::Vector3i(1+4*i, 0+4*i, 2+4*i));  // 0
        _femBodyNode->addFace(Eigen::Vector3i(1+4*i, 2+4*i, 3+4*i));  // 1

        // -- -Z
        _femBodyNode->addFace(Eigen::Vector3i(5+4*i, 6+4*i, 4+4*i));  // 2
        _femBodyNode->addFace(Eigen::Vector3i(5+4*i, 7+4*i, 6+4*i));  // 3

        // -- -Y
        _femBodyNode->addFace(Eigen::Vector3i(0+4*i, 5+4*i, 4+4*i));  // 4
        _femBodyNode->addFace(Eigen::Vector3i(0+4*i, 1+4*i, 5+4*i));  // 5

        // -- +Y
        _femBodyNode->addFace(Eigen::Vector3i(1+4*i, 3+4*i, 7+4*i));  // 6
        _femBodyNode->addFace(Eigen::Vector3i(1+4*i, 7+4*i, 5+4*i));  // 7

        // -- -X
        _femBodyNode->addFace(Eigen::Vector3i(3+4*i, 2+4*i, 6+4*i));  // 8
        _femBodyNode->addFace(Eigen::Vector3i(3+4*i, 6+4*i, 7+4*i));  // 9

        // -- +X
        _femBodyNode->addFace(Eigen::Vector3i(2+4*i, 0+4*i, 4+4*i));  // 10
        _femBodyNode->addFace(Eigen::Vector3i(2+4*i, 4+4*i, 6+4*i));  // 11
    }
    
    //----------------------------------------------------------------------------
    // Tetras
    //----------------------------------------------------------------------------
    for (int i = 0; i < (nPointMasses-4)/4 ; i ++) {
        _femBodyNode->addTetra(1+4*i, 0+4*i, 2+4*i, 4+4*i);
        _femBodyNode->addTetra(1+4*i, 2+4*i, 3+4*i, 7+4*i);
        _femBodyNode->addTetra(2+4*i, 6+4*i, 4+4*i, 7+4*i);
        _femBodyNode->addTetra(2+4*i, 4+4*i, 7+4*i, 1+4*i);
        _femBodyNode->addTetra(1+4*i, 4+4*i, 5+4*i, 7+4*i);
    }
    /*_femBodyNode->addFace(Eigen::Vector3i(0,1,2));
    _femBodyNode->addFace(Eigen::Vector3i(1,2,3));
    _femBodyNode->addFace(Eigen::Vector3i(2,3,0));
    _femBodyNode->addFace(Eigen::Vector3i(3,0,1));
    
    _femBodyNode->addTetra(1, 0, 2, 3);*/
    
    _femBodyNode->postAddingTetra();
    
    std::cout<<"tetra_set\n";
}

void FEMBodyNodeHelper::setBox(FEMBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransfom,
                                const Eigen::Vector3i&   _frags,
                                double                   _totalMass)
{
  assert(_softBodyNode != NULL);
  // Half size
  Eigen::Vector3d halfSize = 0.5 * _size;


  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  assert(_frags[0] > 1 && _frags[1] > 1 && _frags[2] > 1);
  int nVertices = 2 * (_frags[0] * _frags[1])
                     + 2 * (_frags[1] * _frags[2])
                     + 2 * (_frags[2] * _frags[0]);
  Eigen::Vector3d interval(_size[0]/(_frags[0] - 1),
                           _size[1]/(_frags[1] - 1),
                           _size[2]/(_frags[2] - 1));

  // Mass per vertices
  double mass = _totalMass / nVertices;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nVertices,
                                          Eigen::Vector3d::Zero());

  int vIdx = 0;

  // +X side
  for (int k = 0; k < _frags[2]; ++k)    // z
  {
    for (int j = 0; j < _frags[1]; ++j)  // y
    {
      restingPos[vIdx++] <<  halfSize[0],
                            -halfSize[1] + j * interval[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // -X side
  for (int k = 0; k < _frags[2]; ++k)    // z
  {
    for (int j = 0; j < _frags[1]; ++j)  // y
    {
      restingPos[vIdx++] << -halfSize[0],
                            -halfSize[1] + j * interval[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Y side
  for (int i = 0; i < _frags[0]; ++i)    // x
  {
    for (int k = 0; k < _frags[2]; ++k)  // z
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                             halfSize[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Y side
  for (int i = 0; i < _frags[0]; ++i)    // x
  {
    for (int k = 0; k < _frags[2]; ++k)  // z
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1],
                            -halfSize[2] + k * interval[2];
    }
  }

  // +Z side
  for (int j = 0; j < _frags[2]; ++j)    // y
  {
    for (int i = 0; i < _frags[1]; ++i)  // x
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1] + j * interval[1],
                             halfSize[2];
    }
  }

  // -Z side
  for (int j = 0; j < _frags[2]; ++j)    // y
  {
    for (int i = 0; i < _frags[1]; ++i)  // x
    {
      restingPos[vIdx++] << -halfSize[0] + i * interval[0],
                            -halfSize[1] + j * interval[1],
                             halfSize[2];
    }
  }

  // Point masses
  dynamics::FEMPoint* newPointMass = NULL;
  for (int i = 0; i < nVertices; ++i)
  {
    newPointMass = new FEMPoint(_softBodyNode);
    newPointMass->setRestingPosition(_localTransfom * restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int nFaces = 4 * ((_frags[0] - 1) * (_frags[1] - 1))
               + 4 * ((_frags[1] - 1) * (_frags[2] - 1))
               + 4 * ((_frags[2] - 1) * (_frags[0] - 1));
  std::vector<Eigen::Vector3i> faces(nFaces, Eigen::Vector3i::Zero());

  int fIdx = 0;
  int baseIdx = 0;
  Eigen::Vector3i fItr;

  // +X side faces
  for (int k = 0; k < _frags[2] - 1; ++k)    // z
  {
    for (int j = 0; j < _frags[1] - 1; ++j)  // y
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[1] * j + k;
      faces[fIdx][1] = baseIdx + _frags[1] * j + k + 1;
      faces[fIdx][2] = baseIdx + _frags[1] * (j + 1) + k;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[1] * (j + 1) + k + 1;
      faces[fIdx][1] = baseIdx + _frags[1] * (j + 1) + k ;
      faces[fIdx][2] = baseIdx + _frags[1] * j + k + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[1] * _frags[2];

  // -X side faces
  for (int k = 0; k < _frags[2] - 1; ++k)    // z
  {
    for (int j = 0; j < _frags[1] - 1; ++j)  // y
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[1] * j + k;
      faces[fIdx][1] = baseIdx + _frags[1] * (j + 1) + k;
      faces[fIdx][2] = baseIdx + _frags[1] * j + k + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[1] * (j + 1) + k + 1;
      faces[fIdx][1] = baseIdx + _frags[1] * j + k + 1;
      faces[fIdx][2] = baseIdx + _frags[1] * (j + 1) + k ;
      fIdx++;
    }
  }
  baseIdx += _frags[1] * _frags[2];

  // +Y side faces
  for (int i = 0; i < _frags[0] - 1; ++i)    // x
  {
    for (int k = 0; k < _frags[2] - 1; ++k)  // z
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[2] * k + i;
      faces[fIdx][1] = baseIdx + _frags[2] * k + i + 1;
      faces[fIdx][2] = baseIdx + _frags[2] * (k + 1) + i;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[2] * (k + 1) + i + 1;
      faces[fIdx][1] = baseIdx + _frags[2] * (k + 1) + i ;
      faces[fIdx][2] = baseIdx + _frags[2] * k + i + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[2] * _frags[0];

  // -Y side faces
  for (int i = 0; i < _frags[0] - 1; ++i)    // x
  {
    for (int k = 0; k < _frags[2] - 1; ++k)  // z
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[2] * k + i;
      faces[fIdx][1] = baseIdx + _frags[2] * (k + 1) + i;
      faces[fIdx][2] = baseIdx + _frags[2] * k + i + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[2] * (k + 1) + i + 1;
      faces[fIdx][1] = baseIdx + _frags[2] * k + i + 1;
      faces[fIdx][2] = baseIdx + _frags[2] * (k + 1) + i ;
      fIdx++;
    }
  }
  baseIdx += _frags[2] * _frags[0];

  // +Z side faces
  for (int j = 0; j < _frags[1] - 1; ++j)    // y
  {
    for (int i = 0; i < _frags[0] - 1; ++i)  // x
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[0] * i + j;
      faces[fIdx][1] = baseIdx + _frags[0] * i + j + 1;
      faces[fIdx][2] = baseIdx + _frags[0] * (i + 1) + j;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[0] * (i + 1) + j + 1;
      faces[fIdx][1] = baseIdx + _frags[0] * (i + 1) + j ;
      faces[fIdx][2] = baseIdx + _frags[0] * i + j + 1;
      fIdx++;
    }
  }
  baseIdx += _frags[0] * _frags[1];

  // -Z side faces
  for (int j = 0; j < _frags[1] - 1; ++j)    // y
  {
    for (int i = 0; i < _frags[0] - 1; ++i)  // x
    {
      // Lower face
      faces[fIdx][0] = baseIdx + _frags[0] * i + j;
      faces[fIdx][1] = baseIdx + _frags[0] * (i + 1) + j;
      faces[fIdx][2] = baseIdx + _frags[0] * i + j + 1;
      fIdx++;

      // Upper face
      faces[fIdx][0] = baseIdx + _frags[0] * (i + 1) + j + 1;
      faces[fIdx][1] = baseIdx + _frags[0] * i + j + 1;
      faces[fIdx][2] = baseIdx + _frags[0] * (i + 1) + j ;
      fIdx++;
    }
  }

  // Add to the soft body node
  for (int i = 0; i < nFaces; ++i)
  {
    _softBodyNode->addFace(faces[i]);
  }
}

void FEMBodyNodeHelper::setSinglePointMass(FEMBodyNode* _softBodyNode,
                                        double _totalMass)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = 1;\

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = Eigen::Vector3d(+0.1, +0.1, +0.1);

  // Point masses
  dynamics::FEMPoint* newPointMass = NULL;
  for (int i = 0; i < nPointMasses; ++i)
  {
    newPointMass = new FEMPoint(_softBodyNode);
    newPointMass->setRestingPosition(restingPos[i]);
    newPointMass->setMass(mass);
    _softBodyNode->addPointMass(newPointMass);
  }
}

void FEMBodyNodeHelper::setEllipsoid(FEMBodyNode*          _softBodyNode,
                                      const Eigen::Vector3d& _size,
                                      int                    _nSlices,
                                      int                    _nStacks,
                                      double                 _totalMass)
{
  assert(_softBodyNode != NULL);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = (_nStacks - 1) * _nSlices + 2;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Point mass pointer
  FEMPoint* newPointMass = NULL;

  // Resting positions for each point mass
  // -- top
  newPointMass = new dynamics::FEMPoint(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, 0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);
  // middle
  float drho = (DART_PI / _nStacks);
  float dtheta = (DART_2PI / _nSlices);
  for (int i = 1; i < _nStacks; i++)
  {
    float rho = i * drho;
    float srho = (sin(rho));
    float crho = (cos(rho));

    for (int  j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = 0.5 * srho * stheta;
      float y = 0.5 * srho * ctheta;
      float z = 0.5 * crho;

      newPointMass = new dynamics::FEMPoint(_softBodyNode);
      newPointMass->setMass(mass);
      newPointMass->setRestingPosition(
            Eigen::Vector3d(x * _size(0), y * _size(1), z * _size(2)));
      _softBodyNode->addPointMass(newPointMass);
    }
  }
  // bottom
  newPointMass = new dynamics::FEMPoint(_softBodyNode);
  newPointMass->setMass(mass);
  newPointMass->setRestingPosition(Eigen::Vector3d(0.0, 0.0, -0.5 * _size(2)));
  _softBodyNode->addPointMass(newPointMass);



  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  meshIdx1 = 0;
  for (int i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  // middle
  for (int i = 0; i < _nStacks - 2; i++)
  {
    for (int j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = i*_nSlices + j + 1;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = i*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

      meshIdx1 = i*_nSlices + j + 2;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = (i + 1)*_nSlices + j + 2;
      _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
    }

    meshIdx1 = (i + 1)*_nSlices;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = i*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

    meshIdx1 = i*_nSlices + 1;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = (i + 2)*_nSlices + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }

  // bottom
  meshIdx1 = (_nStacks-1)*_nSlices + 1;
  for (int i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = (_nStacks-2)*_nSlices + i + 2;
    meshIdx3 = (_nStacks-2)*_nSlices + i + 1;
    _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = (_nStacks-2)*_nSlices + 2;
  meshIdx3 = (_nStacks-1)*_nSlices;
  _softBodyNode->addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
}

}  // namespace dynamics
}  // namespace dart

