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

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>

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

    floatSeq* vectorToFloatSeq (core::vectorIn_t input)
    {
      ULong size = (ULong) input.size ();
      floatSeq* q_ptr = new floatSeq ();
      q_ptr->length (size);

      for (std::size_t i=0; i<size; ++i) {
        (*q_ptr) [(ULong)i] = input [i];
      }
      return q_ptr;
    }

    floatSeqSeq* matrixToFloatSeqSeq (core::matrixIn_t input)
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

    intSeqSeq* matrixToIntSeqSeq (Eigen::Ref<const IntMatrix_t > input)
    {
      intSeqSeq* res = new intSeqSeq;
      res->length ((ULong) input.rows ());
      for (size_type i=0; i<input.rows (); ++i) {
        intSeq row; row.length ((ULong) input.cols ());
        for (size_type j=0; j<input.cols (); ++j) {
          row [(ULong) j] = input (i, j);
        }
        (*res) [(ULong) i] = row;
      }
      return res;
    }

    vector3_t floatSeqToVector3 (const floatSeq& dofArray)
    {
      if (dofArray.length() != 3) {
        HPP_THROW (Error,
            "Expecting vector of size 3, got vector of size " << dofArray.length() << ".");
      }
      // Fill dof vector with dof array.
      vector3_t result;
      for (unsigned int iDof=0; iDof < 3; ++iDof) {
        result [iDof] = dofArray [iDof];
      }
      return result;
    }

    vector_t floatSeqToVector (const floatSeq& dofArray, const size_type expectedSize)
    {
      size_type inputDim = (size_type)dofArray.length();
      if (expectedSize >=0 && inputDim != expectedSize) {
        HPP_THROW (std::runtime_error,
            "size of input array (" << inputDim <<
            ") is different from the expected size (" << expectedSize << ")");
      }
      // Fill dof vector with dof array.
      vector_t result (inputDim);
      for (size_type iDof=0; iDof < inputDim; ++iDof) {
        result [iDof] = dofArray [(CORBA::ULong)iDof];
      }
      return result;
    }

    Configuration_t floatSeqToConfig (const DevicePtr_t& robot, const floatSeq& dofArray, bool throwIfNotNormalized)
    {
      Configuration_t q (floatSeqToVector (dofArray, robot->configSize()));
      if (throwIfNotNormalized) {
        const value_type eps = std::sqrt(Eigen::NumTraits<value_type>::dummy_precision());
        if (!pinocchio::isNormalized(robot, q, eps))
          throw Error ("Configuration is not normalized (wrong quaternion or complex norm).");
      }
      return q;
    }

    ConfigurationPtr_t floatSeqToConfigPtr (const DevicePtr_t& robot, const floatSeq& dofArray, bool throwIfNotNormalized)
    {
      return ConfigurationPtr_t (new Configuration_t(
            floatSeqToConfig(robot, dofArray, throwIfNotNormalized)
            ));
    }
  } // namespace corbaServer
} // namespace hpp
