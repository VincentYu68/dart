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

#ifndef FEM_UTILS_FEMPARSER_H_
#define FEM_UTILS_FEMPARSER_H_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include <dart/utils/SkelParser.h>

namespace dart {
namespace dynamics {
class Joint;
class FEMBodyNode;
class FEMSkeleton;
}  // namespace dynamics
namespace simulation {
class FEMWorld;
    class FemSimulation;
}  // namespace simulation
}  // namespace dart

namespace dart {
namespace utils {

class FEMSkelParser : public SkelParser
{
public:
  /// \brief
  static simulation::FEMWorld* readFEMFile(const std::string& _filename);

protected:
  /// \brief
  static simulation::FEMWorld* readFEMWorld(tinyxml2::XMLElement* _worldElement);

  /// \brief
  static dynamics::FEMSkeleton* readFEMSkeleton(
      tinyxml2::XMLElement* _femSkeletonElement,
      simulation::World* _world);

  /// \brief
  static SkelBodyNode readFEMBodyNode(
      tinyxml2::XMLElement* _femBodyNodeElement,
      dynamics::FEMSkeleton* _femSkeleton,
      const Eigen::Isometry3d& _skeletonFrame);

  /// \brief
  static dynamics::Joint* readFEMJoint(
      tinyxml2::XMLElement* _jointElement,
      const std::vector<SkelBodyNode,
      Eigen::aligned_allocator<SkelBodyNode> >& _femBodyNodes);
    
  static void setMesh(dynamics::FEMBodyNode*          _femBodyNode,
                                const char *             _filename,
                                const Eigen::Isometry3d& _localTransfom,
                                     double                 _totalMass);
};

}  // namespace utils
}  // namespace dart

#endif  // SOFT_UTILS_SOFTPARSER_H_
