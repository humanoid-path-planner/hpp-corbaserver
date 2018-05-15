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

#include <hpp/corbaserver/object-map.hh>

namespace hpp {
  namespace corbaServer {
      void ObjectMap::createPolyhedron(const std::string name)
	throw (hpp::Error)
      {
	// Check that polyhedron does not already exist.
	if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	polyhedronMap_ [name] = PolyhedronData();
      }

      // --------------------------------------------------------------------

      void ObjectMap::createBox(const std::string name, value_type x, value_type y, value_type z)
	throw (hpp::Error)
      {
	// Check that object does not already exist.
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Box (x, y, z));
      }

      // --------------------------------------------------------------------

      void ObjectMap::createSphere (const std::string name, value_type radius)
	throw (hpp::Error)
      {
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Sphere (radius));
      }

      // --------------------------------------------------------------------

      void ObjectMap::createCylinder (const std::string name, value_type radius, value_type length)
	throw (hpp::Error)
      {
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Cylinder (radius, length));
      }

      // --------------------------------------------------------------------

      std::size_t ObjectMap::addPoint(const std::string name, value_type x, value_type y, value_type z)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
        if (!nameExists<Polyhedron, ThrowIfItDoesNotExist> (name)) return 0;
        PolyhedronData& poly = polyhedronMap_[name];
	poly.pts.push_back (fcl::Vec3f (x, y, z));
	return poly.pts.size () - 1;
      }

      // --------------------------------------------------------------------

      std::size_t ObjectMap::addTriangle(const std::string name,
			       std::size_t pt1, std::size_t pt2, std::size_t pt3)
	throw (hpp::Error)
      {
	// Check that polyhedron exists.
        if (!nameExists<Polyhedron, ThrowIfItDoesNotExist> (name)) return 0;
        PolyhedronData& poly = polyhedronMap_[name];
        poly.tris.push_back (fcl::Triangle (pt1, pt2, pt3));
	return poly.tris.size () - 1;
      }

      // --------------------------------------------------------------------

      CollisionGeometryPtr_t ObjectMap::geometry (const std::string name) throw (Error)
      {
	CollisionGeometryPtr_t geometry;
	// Check that polyhedron exists.
        if (nameExists<Polyhedron, NoThrow> (name)) {
          const PolyhedronData& poly = polyhedronMap_[name];
	  PolyhedronPtr_t polyhedron = PolyhedronPtr_t (new Polyhedron_t);
	  int res = polyhedron->beginModel ();
	  if (res != fcl::BVH_OK) {
	    std::ostringstream oss;
	    oss << "fcl BVHReturnCode = " << res;
	    throw hpp::Error (oss.str ().c_str ());
	  }

	  polyhedron->addSubModel (poly.pts, poly.tris);
	  polyhedron->endModel ();
	  geometry = polyhedron;
	} else if (nameExists<Shape, ThrowIfItDoesNotExist> (name)) {
          geometry = shapeMap_[name];
	}
        return geometry;
      }

      // --------------------------------------------------------------------

      template <ObjectMap::GeomType geomType, ObjectMap::ThrowType throwType>
      bool ObjectMap::nameExists (const std::string& name) const throw (hpp::Error)
      {
        bool exists = false;
        switch (geomType) {
          case Shape:
            exists  = (shapeMap_.find (name) != shapeMap_ .end ());
            break;
          case Polyhedron:
            exists  = (polyhedronMap_.find(name) != polyhedronMap_.end ());
            break;
          case BothGeomType:
            exists  = (shapeMap_.find (name) != shapeMap_ .end ())
                   || (polyhedronMap_.find(name) != polyhedronMap_.end ());
            break;
        }

        switch (throwType) {
          case NoThrow:
            return exists;
          case ThrowIfItExists:
            if (exists) {
              std::ostringstream oss;
              oss << "object " << name << " already exists.";
              throw hpp::Error (oss.str ().c_str ());
            }
            return false;
          case ThrowIfItDoesNotExist:
            if (!exists) {
              std::ostringstream oss;
              oss << "object " << name << " does not exist.";
              throw hpp::Error (oss.str ().c_str ());
            }
            return true;
        }
      }
  } // end of namespace corbaServer.
} // end of namespace hpp.

