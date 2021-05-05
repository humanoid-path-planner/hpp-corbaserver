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

#ifndef HPP_CORBASERVER_CONVERSIONS_HH
#define HPP_CORBASERVER_CONVERSIONS_HH

# include <hpp/util/exception-factory.hh>

# include <hpp/corbaserver/fwd.hh>
# include <hpp/core/parameter.hh>
# include <hpp/common-idl.hh>

namespace hpp {
  namespace corbaServer {
    typedef Eigen::Matrix<CORBA::Long, Eigen::Dynamic, Eigen::Dynamic> IntMatrix_t;

    void toTransform3f (const Transform_ in, Transform3f& out);

    Transform3f toTransform3f (const Transform_ in);

    std::vector<Transform3f> toTransform3f (const TransformSeq in);

    void toHppTransform (const Transform3f& in, Transform_ out);

    Transform__slice* toHppTransform (const Transform3f& in);

    floatSeq* vectorToFloatSeq (core::vectorIn_t input);

    void vectorToFloatSeq (core::vectorIn_t input, floatSeq& output);

    inline floatSeq* vectorToFloatSeq (core::ConfigurationPtr_t input)
    {
      if (!input) return NULL;
      return vectorToFloatSeq (*input);
    }

    /// Returns a sequence of the rows of the input matrix.
    // Return [ [input.row(0)], [input.row(1)], ...]
    floatSeqSeq* matrixToFloatSeqSeq (core::matrixIn_t input);

    intSeqSeq* matrixToIntSeqSeq (Eigen::Ref<const IntMatrix_t > input);

    vector3_t floatSeqToVector3 (const floatSeq& dofArray);

    vector_t floatSeqToVector (const floatSeq& dofArray, const size_type expectedSize = -1);

    Configuration_t floatSeqToConfig (const DevicePtr_t& robot, const floatSeq& dofArray, bool throwIfNotNormalized);

    ConfigurationPtr_t floatSeqToConfigPtr (const DevicePtr_t& robot, const floatSeq& dofArray, bool throwIfNotNormalized);

    core::matrix_t floatSeqSeqToMatrix (const floatSeqSeq& input, const size_type expectedRows = -1, const size_type expectedCols = -1);

    IntMatrix_t intSeqSeqToMatrix (const intSeqSeq& input, const size_type expectedRows = -1, const size_type expectedCols = -1);

    std::vector<bool> boolSeqToVector (const hpp::boolSeq& mask,
                                       unsigned int length = 3);

    inline char* c_str (const std::string& in)
    {
      char* out = new char[in.length()+1];
      strcpy (out, in.c_str());
      return out;
    }

    template <typename InputIt> inline Names_t* toNames_t (InputIt begin, InputIt end)
    {
      std::size_t len = std::distance (begin, end);
      char** nameList = Names_t::allocbuf((CORBA::ULong) len);
      Names_t *ret = new Names_t ((CORBA::ULong) len, (CORBA::ULong) len, nameList, true);

      std::size_t i = 0;
      while (begin != end) {
        nameList[i] = c_str (*begin);
        ++begin;
        ++i;
      }
      return ret;
    }

    template <typename Iterable> inline Names_t* toNames_t (const Iterable& iterable)
    {
      return toNames_t(iterable.begin(), iterable.end());
    }

    template <typename InputIt> inline intSeq* toIntSeq (InputIt begin, InputIt end)
    {
      std::size_t len = std::distance (begin, end);
      intSeq* indexes = new intSeq ();
      indexes->length ((CORBA::ULong) len);

      std::size_t i = 0;
      while (begin != end) {
        (*indexes)[i] = *begin;
        ++begin;
        ++i;
      }
      return indexes;
    }

    template <typename InputIt> inline boolSeq* toBoolSeq (InputIt begin, InputIt end)
    {
      std::size_t len = std::distance (begin, end);
      boolSeq* indexes = new boolSeq ();
      indexes->length ((CORBA::ULong) len);

      std::size_t i = 0;
      while (begin != end) {
        (*indexes)[(CORBA::ULong)i] = *begin;
        ++begin;
        ++i;
      }
      return indexes;
    }

    template <typename OutputType> inline OutputType toStrings (const Names_t& names)
    {
      OutputType ret;
      for (CORBA::ULong i = 0; i < names.length(); ++i)
        ret.push_back (std::string(names[i]));
      return ret;
    }

    /// Convert CORBA comparison types to C++ comparison type.
    constraints::ComparisonTypes_t convertComparison (ComparisonTypes_t comp);

    /// Convert C++ comparison type to CORBA comparison types.
    ComparisonTypes_t* convertComparison (constraints::ComparisonTypes_t comp);

    core::Parameter toParameter (const CORBA::Any& any);

    CORBA::Any toCorbaAny (const core::Parameter& parameter);

    inline CORBA::Any* toCorbaAnyPtr (const core::Parameter& parameter)
    {
      CORBA::Any* ap = new CORBA::Any;
      *ap = toCorbaAny(parameter);
      return ap;
    }

  } // namespace corbaServer
} // namespace hpp

#endif // HPP_CORBASERVER_CONVERSIONS_HH
