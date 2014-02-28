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

#include "dart/utils/FEMParser.h"

#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

#include <dart/common/Console.h>
#include <dart/collision/dart/DARTCollisionDetector.h>
#include <dart/collision/fcl/FCLCollisionDetector.h>
#include <dart/constraint/ConstraintDynamics.h>
// #include <dart/collision/fcl_mesh/FCLMeshCollisionDetector.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/CylinderShape.h>
#include <dart/dynamics/EllipsoidShape.h>
#include <dart/dynamics/WeldJoint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/TranslationalJoint.h>
#include <dart/dynamics/BallJoint.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/simulation/World.h>
#include <dart/utils/SkelParser.h>

#include "dart/collision/fcl_mesh/SoftFCLMeshCollisionDetector.h"
#include "dart/dynamics/FemMeshShape.h"
#include "dart/dynamics/FemBodyNode.h"
#include "dart/dynamics/FemSkeleton.h"
#include "dart/simulation/FemWorld.h"
#include "dart/simulation/FemSim.h"

namespace dart {
namespace utils {

simulation::FEMWorld* FEMSkelParser::readFEMFile(
    const std::string& _filename)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, _filename.c_str());
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile [" << _filename << "] Fails: "
              << e.what() << std::endl;
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* skelElement = NULL;
  skelElement = _dartFile.FirstChildElement("skel");
  if (skelElement == NULL)
  {
    dterr << "Skel file[" << _filename << "] does not contain <skel> as the "
          << "element.\n";
    return NULL;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = NULL;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == NULL)
  {
    dterr << "Skel file[" << _filename << "] does not contain <world> element "
          <<"under <skel> element.\n";
    return NULL;
  }

  simulation::FEMWorld* newWorld = readFEMWorld(worldElement);

  return newWorld;
}

simulation::FEMWorld* FEMSkelParser::readFEMWorld(
    tinyxml2::XMLElement* _worldElement)
{
  assert(_worldElement != NULL);

  // Create a world
  simulation::FEMWorld* newFEMWorld = new simulation::FEMWorld;

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement
      = _worldElement->FirstChildElement("physics");
  if (physicsElement != NULL)
  {
    // Time step
    tinyxml2::XMLElement* timeStepElement = NULL;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != NULL)
    {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newFEMWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = NULL;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != NULL)
    {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newFEMWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector"))
    {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh")
      {
        newFEMWorld->getConstraintHandler()->setCollisionDetector(
              new collision::SoftFCLMeshCollisionDetector());
      }
      else if (strCD == "fcl")
      {
        newFEMWorld->getConstraintHandler()->setCollisionDetector(
              new collision::FCLCollisionDetector());
      }
      else if (strCD == "dart")
      {
        newFEMWorld->getConstraintHandler()->setCollisionDetector(
              new collision::DARTCollisionDetector());
      }
      else
      {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    }
    else
    {
      newFEMWorld->getConstraintHandler()->setCollisionDetector(
            new collision::SoftFCLMeshCollisionDetector());
    }
  }

  //--------------------------------------------------------------------------
  // Load soft skeletons
  ElementEnumerator femSkeletonElements(_worldElement, "skeleton");
  while (femSkeletonElements.next())
  {
    dynamics::FEMSkeleton* newFEMSkeleton
        = readFEMSkeleton(femSkeletonElements.get(), newFEMWorld);

    newFEMWorld->addSkeleton(newFEMSkeleton);
  }

    newFEMWorld->gatherFemSim();
    
  return newFEMWorld;
}

dynamics::FEMSkeleton* FEMSkelParser::readFEMSkeleton(
    tinyxml2::XMLElement *_femSkeletonElement,
    simulation::World *_femWorld)
{
  assert(_femSkeletonElement != NULL);
  assert(_femWorld != NULL);

  dynamics::FEMSkeleton* newFEMSkeleton = new dynamics::FEMSkeleton;
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_femSkeletonElement, "name");
  newFEMSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_femSkeletonElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_femSkeletonElement, "transformation");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // immobile attribute
//  tinyxml2::XMLElement* immobileElement = NULL;
//  immobileElement = _softSkeletonElement->FirstChildElement("immobile");
//  if (immobileElement != NULL)
//  {
//    std::string stdImmobile = immobileElement->GetText();
//    bool immobile = toBool(stdImmobile);
//    newSoftSkeleton->setImmobileState(immobile);
//  }

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator bodies(_femSkeletonElement, "body");
  std::vector<SkelBodyNode, Eigen::aligned_allocator<SkelBodyNode> >
      femBodyNodes;
  while (bodies.next())
  {
    SkelBodyNode newFEMBodyNode
        = readFEMBodyNode(bodies.get(),
                           newFEMSkeleton,
                           skeletonFrame);
    assert(newFEMBodyNode.bodyNode);
    femBodyNodes.push_back(newFEMBodyNode);
  }

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator joints(_femSkeletonElement, "joint");
  while (joints.next())
    readFEMJoint(joints.get(), femBodyNodes);

  //--------------------------------------------------------------------------
  // Add FreeJoint to the body node that doesn't have parent joint
//  for (unsigned int i = 0; i < skelBodyNodes.size(); ++i)
//  {
//    dynamics::BodyNode* bodyNode = skelBodyNodes[i].bodyNode;

//    if (bodyNode->getParentJoint() == NULL)
//    {
//      // If this link has no parent joint, then we add 6-dof free joint.
//      dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

//      newFreeJoint->setTransformFromParentBodyNode(
//            bodyNode->getWorldTransform());
//      newFreeJoint->setTransformFromChildBodyNode(
//            Eigen::Isometry3d::Identity());

//      bodyNode->setParentJoint(newFreeJoint);
//    }
//  }

  for (std::vector<SkelBodyNode,
       Eigen::aligned_allocator<SkelBodyNode> >::iterator it =
       femBodyNodes.begin();
       it != femBodyNodes.end(); ++it)
  {
    dynamics::FEMBodyNode* fem
        = dynamic_cast<dynamics::FEMBodyNode*>((*it).bodyNode);
    if (fem)
      newFEMSkeleton->addFEMBodyNode(fem);
    else
      newFEMSkeleton->addBodyNode((*it).bodyNode);
  }

  return newFEMSkeleton;
}

SkelParser::SkelBodyNode FEMSkelParser::readFEMBodyNode(
    tinyxml2::XMLElement*    _femBodyNodeElement,
    dynamics::FEMSkeleton*  _femSkeleton,
    const Eigen::Isometry3d& _skeletonFrame)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(_femBodyNodeElement != NULL);
  assert(_femSkeleton != NULL);

  // If _softBodyNodeElement has no <fem_shape>, return rigid body node
  if (!hasElement(_femBodyNodeElement, "fem_shape"))
  {
    return SkelParser::readBodyNode(_femBodyNodeElement,
                                    _femSkeleton,
                                    _skeletonFrame);
  }
    
    

  dynamics::FEMBodyNode* newFEMBodyNode = new dynamics::FEMBodyNode;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttribute(_femBodyNodeElement, "name");
  newFEMBodyNode->setName(name);

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_femBodyNodeElement, "gravity"))
  {
    bool gravityMode = getValueBool(_femBodyNodeElement, "gravity");
    newFEMBodyNode->setGravityMode(gravityMode);
  }

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_femBodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(_femBodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  }
  else
  {
    initTransform = _skeletonFrame;
  }

  // visualization_shape
  if (hasElement(_femBodyNodeElement, "visualization_shape"))
  {
    tinyxml2::XMLElement* vizElement
        = getElement(_femBodyNodeElement, "visualization_shape");

    dynamics::Shape* shape = NULL;

    // type
    assert(hasElement(vizElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = new dynamics::CylinderShape(radius, height);
    }
    else
    {
      dterr << "Unknown visualization shape.\n";
      assert(0);
    }
    newFEMBodyNode->addVisualizationShape(shape);

    // transformation
    if (hasElement(vizElement, "transformation"))
    {
      Eigen::Isometry3d W = getValueIsometry3d(vizElement, "transformation");
      shape->setLocalTransform(W);
    }
  }

  // collision_shape
  if (hasElement(_femBodyNodeElement, "collision_shape"))
  {
    tinyxml2::XMLElement* colElement
        = getElement(_femBodyNodeElement, "collision_shape");

    dynamics::Shape* shape = NULL;

    // type
    assert(hasElement(colElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(colElement, "geometry");

    // FIXME: Assume that type has only one shape type.
    if (hasElement(geometryElement, "box"))
    {
      tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

      Eigen::Vector3d size = getValueVector3d(boxElement, "size");

      shape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement,
                                                          "ellipsoid");

      Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

      shape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
      tinyxml2::XMLElement* cylinderElement = getElement(geometryElement,
                                                         "cylinder");

      double radius = getValueDouble(cylinderElement, "radius");
      double height = getValueDouble(cylinderElement, "height");

      shape = new dynamics::CylinderShape(radius, height);
    }
    else
    {
      dterr << "Unknown visualization shape.\n";
      assert(0);
    }
    newFEMBodyNode->addCollisionShape(shape);

    // transformation
    if (hasElement(colElement, "transformation"))
    {
      Eigen::Isometry3d W = getValueIsometry3d(colElement, "transformation");
      shape->setLocalTransform(W);
    }
  }

  //----------------------------------------------------------------------------
  // Soft properties
  if (hasElement(_femBodyNodeElement, "fem_shape"))
  {
      simulation::FemSimulation* femsim = new simulation::FemSimulation();
      newFEMBodyNode->setFEMSim(femsim);
      
    tinyxml2::XMLElement* femShapeEle
        = getElement(_femBodyNodeElement, "fem_shape");

    // mass
    double totalMass = getValueDouble(femShapeEle, "total_mass");

    // transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(femShapeEle, "transformation"))
      T = getValueIsometry3d(femShapeEle, "transformation");

      if (hasElement(femShapeEle, "ym") && hasElement(femShapeEle, "pr")) {
          double ym = getValueDouble(femShapeEle, "ym");
          double pr = getValueDouble(femShapeEle, "pr");
          
          femsim->setParameters(ym, pr);
      }
      
    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(femShapeEle, "geometry");
    if (hasElement(geometryEle, "box"))
    {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size  = getValueVector3d(boxEle, "size");
//      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      dynamics::FEMBodyNodeHelper::setBox(newFEMBodyNode, size, T, totalMass);
//      dynamics::SoftBodyNodeHelper::setBox(newSoftBodyNode, size, frags,
//                                           totalMass);

      // Visualization shape
      newFEMBodyNode->addVisualizationShape(
            new dynamics::FEMMeshShape(newFEMBodyNode));

      // Collision shape
      newFEMBodyNode->addCollisionShape(
            new dynamics::FEMMeshShape(newFEMBodyNode));
    }
    else if (hasElement(geometryEle, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      double nSlices       = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks       = getValueDouble(ellipsoidEle, "num_stacks");
      dynamics::FEMBodyNodeHelper::setEllipsoid(newFEMBodyNode,
                                                 size,
                                                 nSlices,
                                                 nStacks,
                                                 totalMass);

      // Visualization shape
      newFEMBodyNode->addVisualizationShape(
            new dynamics::FEMMeshShape(newFEMBodyNode));

      // Collision shape
      newFEMBodyNode->addCollisionShape(
            new dynamics::FEMMeshShape(newFEMBodyNode));
    }
    else
    {
      dterr << "Unknown soft shape.\n";
    }

  }

  SkelBodyNode femBodyNode;
  femBodyNode.bodyNode = newFEMBodyNode;
  femBodyNode.initTransform = initTransform;

  return femBodyNode;
}

dynamics::Joint* FEMSkelParser::readFEMJoint(
    tinyxml2::XMLElement* _jointElement,
    const std::vector<SkelBodyNode,
    Eigen::aligned_allocator<SkelBodyNode> >& _femBodyNodes)
{
  assert(_jointElement != NULL);

  dynamics::Joint* newJoint = NULL;

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(_jointElement, "type");
  assert(!type.empty());
  if (type == std::string("weld"))
    newJoint = SkelParser::readWeldJoint(_jointElement);
  if (type == std::string("revolute"))
    newJoint = SkelParser::readRevoluteJoint(_jointElement);
  if (type == std::string("prismatic"))
    newJoint = SkelParser::readPrismaticJoint(_jointElement);
  if (type == std::string("ball"))
    newJoint = SkelParser::readBallJoint(_jointElement);
  if (type == std::string("translational"))
    newJoint = SkelParser::readTranslationalJoint(_jointElement);
  if (type == std::string("free"))
    newJoint = SkelParser::readFreeJoint(_jointElement);
  assert(newJoint != NULL);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");
  newJoint->setName(name);

  //--------------------------------------------------------------------------
  // parent
  SkelBodyNode femParentBodyNode;
  femParentBodyNode.bodyNode = NULL;
  femParentBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "parent"))
  {
    std::string strParent = getValueString(_jointElement, "parent");

    if (strParent != std::string("world"))
    {
      for (std::vector<SkelBodyNode,
           Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
           _femBodyNodes.begin(); it != _femBodyNodes.end(); ++it)
        if ((*it).bodyNode->getName() == strParent)
        {
          femParentBodyNode = (*it);
          break;
        }

      if (femParentBodyNode.bodyNode == NULL)
      {
        dterr << "Can't find the parent body ["
              << strParent
              << "] of the joint ["
              << newJoint->getName()
              << "]. " << std::endl;
        assert(0);
      }
    }
  }
  else
  {
    dterr << "No parent body.\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // child
  SkelBodyNode femChildBodyNode;
  femChildBodyNode.bodyNode = NULL;
  femChildBodyNode.initTransform = Eigen::Isometry3d::Identity();

  if (hasElement(_jointElement, "child"))
  {
    std::string strChild = getValueString(_jointElement, "child");

    for (std::vector<SkelBodyNode,
         Eigen::aligned_allocator<SkelBodyNode> >::const_iterator it =
         _femBodyNodes.begin(); it != _femBodyNodes.end(); ++it)
    {
      if ((*it).bodyNode->getName() == strChild)
      {
        femChildBodyNode = (*it);
        break;
      }
    }

    if (femChildBodyNode.bodyNode == NULL)
    {
      dterr << "Can't find the child body ["
            << strChild
            << "] of the joint ["
            << newJoint->getName()
            << "]. " << std::endl;
      assert(0);
    }
  }
  else
  {
    dterr << "Set child body node for " << newJoint->getName() << "."
          << std::endl;
    assert(0);
  }

  femChildBodyNode.bodyNode->setParentJoint(newJoint);

  if (femParentBodyNode.bodyNode)
    femParentBodyNode.bodyNode->addChildBodyNode(
          femChildBodyNode.bodyNode);

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = femChildBodyNode.initTransform;
  if (femParentBodyNode.bodyNode)
    parentWorld = femParentBodyNode.initTransform;
  if (hasElement(_jointElement, "transformation"))
    childToJoint = getValueIsometry3d(_jointElement, "transformation");
  Eigen::Isometry3d parentToJoint =
      parentWorld.inverse()*childWorld*childToJoint;
  newJoint->setTransformFromChildBodyNode(childToJoint);
  newJoint->setTransformFromParentBodyNode(parentToJoint);

  return newJoint;
}

}  // namespace utils
}  // namespace dart
