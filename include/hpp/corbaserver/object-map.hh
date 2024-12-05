// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_CORBASERVER_OBJECT_MAP_HH
#define HPP_CORBASERVER_OBJECT_MAP_HH

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <hpp/common-idl.hh>
#include <hpp/corbaserver/fwd.hh>
#include <pinocchio/fwd.hpp>

namespace hpp {
namespace corbaServer {
class ObjectMap {
 public:
  void createBox(const std::string boxName, value_type x, value_type y,
                 value_type z);
  void createSphere(const std::string name, value_type radius);
  void createCylinder(const std::string name, value_type radius,
                      value_type length);

  void createPolyhedron(const std::string polyhedronName);
  std::size_t addPoint(const std::string polyhedronName, value_type x,
                       value_type y, value_type z);
  std::size_t addTriangle(const std::string polyhedronName, std::size_t pt1,
                          std::size_t pt2, std::size_t pt3);

  CollisionGeometryPtr_t geometry(const std::string name) /*const*/;

 protected:
  struct PolyhedronData {
    std::vector<coal::Vec3f> pts;
    std::vector<coal::Triangle> tris;
  };
  typedef std::map<std::string, PolyhedronData> PolyhedronMap_t;
  typedef std::map<std::string, CollisionGeometryPtr_t> ShapeMap_t;

  enum GeomType { Shape = 1, Polyhedron = 2, BothGeomType = 3 };

  enum ThrowType { NoThrow, ThrowIfItExists, ThrowIfItDoesNotExist };

  template <GeomType geomType, ThrowType throwType>
  bool nameExists(const std::string& name) const;

  /// Map of basic shapes
  ShapeMap_t shapeMap_;
  /// Map of polyhedra in construction.
  PolyhedronMap_t polyhedronMap_;
};
}  // end of namespace corbaServer.
}  // end of namespace hpp.

#endif  // HPP_CORBASERVER_OBJECT_MAP_HH
