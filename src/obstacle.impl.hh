// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_OBSTACLE_IMPL_HH
#define HPP_CORBASERVER_OBSTACLE_IMPL_HH
#include <hpp/fcl/data_types.h>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/object-map.hh>
#include <hpp/corbaserver/obstacle-idl.hh>
#include <hpp/core/problem-solver.hh>
#include <map>
#include <string>

/// \brief Implement CORBA interface ``Obstacle''.
namespace hpp {
namespace corbaServer {
namespace impl {
class Obstacle : public virtual POA_hpp::corbaserver::Obstacle {
 public:
  Obstacle();

  void setServer(ServerPlugin* server) { server_ = server; }

  virtual void loadObstacleModel(const char* filename, const char* prefix);

  virtual void loadObstacleModelFromString(const char* urdfString,
                                           const char* prefix);

  virtual void loadPolyhedron(const char* name, const char* filename);

  virtual void removeObstacleFromJoint(const char* objectName,
                                       const char* jointName, Boolean collision,
                                       Boolean distance);

  virtual void removeObstacle(const char* objectName);

  virtual void cutObstacle(const char* objectName, const floatSeq& aabb);

  virtual void addObstacle(const char* polyhedronName, Boolean collision,
                           Boolean distance);

  virtual void moveObstacle(const char* polyName, const Transform_ cfg);

  virtual void getObstaclePosition(const char* objectName, Transform_ cfg);

  virtual Names_t* getObstacleNames(bool collision, bool distance);

  virtual void createPolyhedron(const char* polyhedronName);

  virtual void createBox(const char* boxName, Double x, Double y, Double z);

  virtual void createSphere(const char* name, Double radius);

  virtual void createCylinder(const char* name, Double radius, Double length);

  virtual ULong addPoint(const char* polyhedronName, Double x, Double y,
                         Double z);

  virtual ULong addTriangle(const char* polyhedronName, ULong pt1, ULong pt2,
                            ULong pt3);

 private:
  CollisionObjectPtr_t getObstacleByName(const char* name);

  /// Map of object in construction.
  ObjectMap objectMap_;

  /// \brief Pointer to the ServerPlugin owning this object.
  ServerPlugin* server_;

  /// \brief Pointer to hpp::core::ProblemSolver object of ServerPlugin.
  core::ProblemSolverPtr_t problemSolver();
};

}  // namespace impl
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  //! HPP_CORBASERVER_OBSTACLE_HH
