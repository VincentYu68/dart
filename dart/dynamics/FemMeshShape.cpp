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

#include "dart/dynamics/FemMeshShape.h"

#include <dart/common/Console.h>

#include "dart/dynamics/FEMPoint.h"
#include "dart/dynamics/FEMBodyNode.h"

namespace dart {
namespace dynamics {

FEMMeshShape::FEMMeshShape(FEMBodyNode* _femBodyNode)
  : Shape(SOFT_MESH),
    mAssimpMesh(NULL),
    mFEMBodyNode(_femBodyNode)
{
  assert(_softBodyNode != NULL);
  // Build mesh here using soft body node
  // TODO(JS): Not implemented.
  _buildMesh();
}

FEMMeshShape::~FEMMeshShape()
{
  if (mAssimpMesh != NULL)
    delete mAssimpMesh;
}

const aiMesh* FEMMeshShape::getAssimpMesh() const
{
  return mAssimpMesh;
}

Eigen::Matrix3d FEMMeshShape::computeInertia(double _mass) const
{
  // TODO(JS): Not implemented.
  return Eigen::Matrix3d();
}

void FEMMeshShape::draw(renderer::RenderInterface* _ri,
                         const Eigen::Vector4d&     _col,
                         bool                       _default) const
{
  // TODO(JS): Not implemented.
}

void FEMMeshShape::computeVolume()
{
  // TODO(JS): Not implemented.
}

void FEMMeshShape::_buildMesh()
{
  // Get number of vertices and faces from soft body node
  int nVertices = mFEMBodyNode->getNumPointMasses();
  int nFaces    = mFEMBodyNode->getNumFaces();

  // Create new aiMesh
  mAssimpMesh = new aiMesh();

  // Set vertices and normals
  mAssimpMesh->mNumVertices = nVertices;
  mAssimpMesh->mVertices    = new aiVector3D[nVertices];
  mAssimpMesh->mNormals     = new aiVector3D[nVertices];
  aiVector3D itAIVector3d;
  for (int i = 0; i < nVertices; ++i)
  {
    FEMPoint* itPointMass        = mFEMBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getRestingPosition();
    itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
    mAssimpMesh->mVertices[i] = itAIVector3d;
    mAssimpMesh->mNormals[i]  = itAIVector3d;
  }

  // Set faces
  mAssimpMesh->mNumFaces = nFaces;
  mAssimpMesh->mFaces = new aiFace[nFaces];
  for (int i = 0; i < nFaces; ++i)
  {
    Eigen::Vector3i itFace = mFEMBodyNode->getFace(i);
    aiFace* itAIFace = &mAssimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices    = new unsigned int[3];
    itAIFace->mIndices[0] = itFace[0];
    itAIFace->mIndices[1] = itFace[1];
    itAIFace->mIndices[2] = itFace[2];
  }
}

void FEMMeshShape::update()
{
  int nVertices = mFEMBodyNode->getNumPointMasses();

  aiVector3D itAIVector3d;
  for (int i = 0; i < nVertices; ++i)
  {
    FEMPoint* itPointMass        = mFEMBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getPosition();
    itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
    mAssimpMesh->mVertices[i] = itAIVector3d;
  }
}

}  // namespace dynamics
}  // namespace dart
