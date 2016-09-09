// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/corbaserver/conversions.hh>

#include <pinocchio/spatial/se3.hpp>
#include <hpp/corbaserver/config.hh>

namespace hpp {
  namespace corbaServer {
    using CORBA::ULong;

    void toTransform3f (const Transform_ in, Transform3f& out)
    {
      Transform3f::Quaternion_t Q (in [6], in [3], in [4], in [5]);
      out.translation() << in [0], in [1], in [2];
      out.rotation() = Q.matrix();
    }

    Transform3f toTransform3f (const Transform_ in)
    {
      Transform3f out;
      toTransform3f (in, out);
      return out;
    }

    void toHppTransform (const Transform3f& in, Transform_ out)
    {
      Transform3f::Quaternion_t q (in.rotation());
      out[3] = q.x();
      out[4] = q.y();
      out[5] = q.z();
      out[6] = q.w();
      for(int i=0; i<3; i++)
        out[i] = in.translation() [i];
    }

    Transform__slice* toHppTransform (const Transform3f& in)
    {
      Transform__slice* out = Transform__alloc();
      toHppTransform (in, out);
      return out;
    }

    floatSeq* vectorToFloatseq (const vector_t& input)
    {
      ULong size = (ULong) input.size ();
      floatSeq* q_ptr = new floatSeq ();
      q_ptr->length (size);

      for (std::size_t i=0; i<size; ++i) {
        (*q_ptr) [(ULong)i] = input [i];
      }
      return q_ptr;
    }

    floatSeqSeq* matrixToFloatSeqSeq (const matrix_t& input)
    {
      floatSeqSeq* res = new floatSeqSeq;
      res->length ((ULong) input.rows ());
      for (size_type i=0; i<input.rows (); ++i) {
        floatSeq row; row.length ((ULong) input.cols ());
        for (size_type j=0; j<input.cols (); ++j) {
          row [(ULong) j] = input (i, j);
        }
        (*res) [(ULong) i] = row;
      }
      return res;
    }
  } // namespace corbaServer
} // namespace hpp
