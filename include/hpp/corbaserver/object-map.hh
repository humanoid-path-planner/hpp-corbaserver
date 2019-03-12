// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-corbaserver.
// hpp-corbaserver is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-corbaserver is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-corbaserver. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORBASERVER_OBJECT_MAP_HH
# define HPP_CORBASERVER_OBJECT_MAP_HH

# include <hpp/fcl/BVH/BVH_model.h>
# include <hpp/fcl/shape/geometric_shapes.h>

# include <hpp/corbaserver/fwd.hh>
# include <hpp/common-idl.hh>

namespace hpp {
  namespace corbaServer {
    class ObjectMap
    {
      public:
        void createBox (const std::string boxName, value_type x, value_type y, value_type z) throw (Error);
        void createSphere (const std::string name, value_type radius) throw (Error);
        void createCylinder (const std::string name, value_type radius, value_type length) throw (Error);

        void createPolyhedron (const std::string polyhedronName) throw (hpp::Error);
        std::size_t addPoint (const std::string polyhedronName, value_type x, value_type y, value_type z) throw (hpp::Error);
        std::size_t addTriangle (const std::string polyhedronName, std::size_t pt1, std::size_t pt2, std::size_t pt3) throw (hpp::Error);

        CollisionGeometryPtr_t geometry (const std::string name) /*const*/ throw (Error);

      protected:
        struct PolyhedronData {
          std::vector <fcl::Vec3f> pts;
          std::vector <fcl::Triangle> tris;
        };
        typedef std::map <std::string, PolyhedronData> PolyhedronMap_t;
        typedef std::map <std::string, BasicShapePtr_t> ShapeMap_t;

        enum GeomType {
          Shape = 1,
          Polyhedron = 2,
          BothGeomType = 3
        };

        enum ThrowType {
          NoThrow,
          ThrowIfItExists,
          ThrowIfItDoesNotExist
        };

        template <GeomType geomType, ThrowType throwType>
        bool nameExists (const std::string& name) const throw (hpp::Error);

        /// Map of basic shapes
        ShapeMap_t shapeMap_;
        /// Map of polyhedra in construction.
        PolyhedronMap_t polyhedronMap_;
    };
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_CORBASERVER_OBJECT_MAP_HH
