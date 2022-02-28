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

#include <hpp/corbaserver/object-map.hh>

namespace hpp {
  namespace corbaServer {
      void ObjectMap::createPolyhedron(const std::string name)
      {
	// Check that polyhedron does not already exist.
	if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	polyhedronMap_ [name] = PolyhedronData();
      }

      // --------------------------------------------------------------------

      void ObjectMap::createBox(const std::string name, value_type x, value_type y, value_type z)
      {
	// Check that object does not already exist.
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Box (x, y, z));
      }

      // --------------------------------------------------------------------

      void ObjectMap::createSphere (const std::string name, value_type radius)
      {
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Sphere (radius));
      }

      // --------------------------------------------------------------------

      void ObjectMap::createCylinder (const std::string name, value_type radius, value_type length)
      {
        if (nameExists<BothGeomType, ThrowIfItExists> (name)) return;
	shapeMap_ [name] = BasicShapePtr_t (new fcl::Cylinder (radius, length));
      }

      // --------------------------------------------------------------------

      std::size_t ObjectMap::addPoint(const std::string name, value_type x, value_type y, value_type z)
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
      {
	// Check that polyhedron exists.
        if (!nameExists<Polyhedron, ThrowIfItDoesNotExist> (name)) return 0;
        PolyhedronData& poly = polyhedronMap_[name];
        poly.tris.push_back (fcl::Triangle (pt1, pt2, pt3));
	return poly.tris.size () - 1;
      }

      // --------------------------------------------------------------------

      CollisionGeometryPtr_t ObjectMap::geometry (const std::string name)
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
      bool ObjectMap::nameExists (const std::string& name) const
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

