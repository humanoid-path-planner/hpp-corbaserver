// Copyright (c) 2016, Joseph Mirabel
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

#ifndef HPP_CORBASERVER_CONVERSIONS_HH
#define HPP_CORBASERVER_CONVERSIONS_HH

#include <hpp/common-idl.hh>
#include <hpp/corbaserver/fwd.hh>
#include <hpp/core/parameter.hh>
#include <hpp/util/exception-factory.hh>

namespace hpp {
namespace corbaServer {
typedef Eigen::Matrix<CORBA::Long, Eigen::Dynamic, Eigen::Dynamic> IntMatrix_t;

void toTransform3s(const Transform_ in, Transform3s& out);

Transform3s toTransform3s(const Transform_ in);

std::vector<Transform3s> toTransform3s(const TransformSeq in);

void toHppTransform(const Transform3s& in, Transform_ out);

Transform__slice* toHppTransform(const Transform3s& in);

void toIntSeq(std::vector<int> const& in, intSeq& out);

floatSeq* vectorToFloatSeq(core::vectorIn_t input);

void vectorToFloatSeq(core::vectorIn_t input, floatSeq& output);

/// Returns a sequence of the rows of the input matrix.
// Return [ [input.row(0)], [input.row(1)], ...]
floatSeqSeq* matrixToFloatSeqSeq(core::matrixIn_t input);

intSeqSeq* matrixToIntSeqSeq(Eigen::Ref<const IntMatrix_t> input);

std::vector<int> intSeqToVector(const intSeq& dofArray);

vector3_t floatSeqToVector3(const floatSeq& dofArray);

vector_t floatSeqToVector(const floatSeq& dofArray,
                          const size_type expectedSize = -1);

Configuration_t floatSeqToConfig(const DevicePtr_t& robot,
                                 const floatSeq& dofArray,
                                 bool throwIfNotNormalized);

core::matrix_t floatSeqSeqToMatrix(const floatSeqSeq& input,
                                   const size_type expectedRows = -1,
                                   const size_type expectedCols = -1);

IntMatrix_t intSeqSeqToMatrix(const intSeqSeq& input,
                              const size_type expectedRows = -1,
                              const size_type expectedCols = -1);

std::vector<bool> boolSeqToVector(const hpp::boolSeq& mask,
                                  unsigned int length = 3);

stringSeqSeq* vectorToStringSeqSeq(std::vector<std::vector<std::string>> input);

inline char* c_str(const std::string& in) {
  char* out = new char[in.length() + 1];
  strcpy(out, in.c_str());
  return out;
}

template <typename InputIt>
inline Names_t* toNames_t(InputIt begin, InputIt end) {
  std::size_t len = std::distance(begin, end);
  char** nameList = Names_t::allocbuf((CORBA::ULong)len);
  Names_t* ret =
      new Names_t((CORBA::ULong)len, (CORBA::ULong)len, nameList, true);

  std::size_t i = 0;
  while (begin != end) {
    nameList[i] = c_str(*begin);
    ++begin;
    ++i;
  }
  return ret;
}

template <typename Iterable>
inline Names_t* toNames_t(const Iterable& iterable) {
  return toNames_t(iterable.begin(), iterable.end());
}

template <typename InputIt>
inline intSeq* toIntSeq(InputIt begin, InputIt end) {
  std::size_t len = std::distance(begin, end);
  intSeq* indexes = new intSeq();
  indexes->length((CORBA::ULong)len);

  std::size_t i = 0;
  while (begin != end) {
    (*indexes)[(CORBA::ULong)i] = *begin;
    ++begin;
    ++i;
  }
  return indexes;
}

intSeq* toIntSeq(std::vector<int> const& in);

template <typename InputIt>
inline boolSeq* toBoolSeq(InputIt begin, InputIt end) {
  std::size_t len = std::distance(begin, end);
  boolSeq* indexes = new boolSeq();
  indexes->length((CORBA::ULong)len);

  std::size_t i = 0;
  while (begin != end) {
    (*indexes)[(CORBA::ULong)i] = *begin;
    ++begin;
    ++i;
  }
  return indexes;
}

template <typename OutputType>
inline OutputType toStrings(const Names_t& names) {
  OutputType ret;
  for (CORBA::ULong i = 0; i < names.length(); ++i)
    ret.push_back(std::string(names[i]));
  return ret;
}

/// Convert CORBA comparison types to C++ comparison type.
constraints::ComparisonTypes_t convertComparison(hpp::ComparisonTypes_t comp);

/// Convert C++ comparison type to CORBA comparison types.
hpp::ComparisonTypes_t* convertComparison(constraints::ComparisonTypes_t comp);

core::Parameter toParameter(const CORBA::Any& any);

CORBA::Any toCorbaAny(const core::Parameter& parameter);

inline CORBA::Any* toCorbaAnyPtr(const core::Parameter& parameter) {
  CORBA::Any* ap = new CORBA::Any;
  *ap = toCorbaAny(parameter);
  return ap;
}

}  // namespace corbaServer
}  // namespace hpp

#endif  // HPP_CORBASERVER_CONVERSIONS_HH
