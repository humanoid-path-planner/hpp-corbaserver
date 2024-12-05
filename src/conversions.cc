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

#include <boost/mpl/for_each.hpp>
#include <boost/mpl/vector.hpp>
#include <hpp/constraints/comparison-types.hh>
#include <hpp/corbaserver/config.hh>
#include <hpp/corbaserver/conversions.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
namespace corbaServer {
using CORBA::ULong;

void toTransform3s(const Transform_ in, Transform3s& out) {
  Transform3s::Quaternion Q(in[6], in[3], in[4], in[5]);
  const value_type eps = 1e-8;
  if (std::fabs(Q.squaredNorm() - 1) > eps)
    throw Error("Quaternion is not normalized.");
  out.translation() << in[0], in[1], in[2];
  out.rotation() = Q.matrix();
}

Transform3s toTransform3s(const Transform_ in) {
  Transform3s out;
  toTransform3s(in, out);
  return out;
}

std::vector<Transform3s> toTransform3s(const TransformSeq in) {
  std::vector<Transform3s> out(in.length());
  for (std::size_t i = 0; i < out.size(); ++i)
    toTransform3s(in[(ULong)i], out[i]);
  return out;
}

void toHppTransform(const Transform3s& in, Transform_ out) {
  Transform3s::Quaternion q(in.rotation());
  out[3] = q.x();
  out[4] = q.y();
  out[5] = q.z();
  out[6] = q.w();
  for (int i = 0; i < 3; i++) out[i] = in.translation()[i];
}

Transform__slice* toHppTransform(const Transform3s& in) {
  Transform__slice* out = Transform__alloc();
  toHppTransform(in, out);
  return out;
}

void toIntSeq(std::vector<int> const& input, intSeq& output) {
  ULong size = (ULong)input.size();
  output.length(size);

  for (std::size_t i = 0; i < size; ++i) {
    output[(ULong)i] = input[i];
  }
}

intSeq* toIntSeq(std::vector<int> const& in) {
  return toIntSeq(in.begin(), in.end());
}

floatSeq* vectorToFloatSeq(core::vectorIn_t input) {
  floatSeq* q_ptr = new floatSeq();
  vectorToFloatSeq(input, *q_ptr);
  return q_ptr;
}

void vectorToFloatSeq(core::vectorIn_t input, floatSeq& output) {
  ULong size = (ULong)input.size();
  output.length(size);

  for (std::size_t i = 0; i < size; ++i) {
    output[(ULong)i] = input[i];
  }
}

floatSeqSeq* matrixToFloatSeqSeq(core::matrixIn_t input) {
  floatSeqSeq* res = new floatSeqSeq;
  res->length((ULong)input.rows());
  for (size_type i = 0; i < input.rows(); ++i) {
    floatSeq row;
    row.length((ULong)input.cols());
    for (size_type j = 0; j < input.cols(); ++j) {
      row[(ULong)j] = input(i, j);
    }
    (*res)[(ULong)i] = row;
  }
  return res;
}

intSeqSeq* matrixToIntSeqSeq(Eigen::Ref<const IntMatrix_t> input) {
  intSeqSeq* res = new intSeqSeq;
  res->length((ULong)input.rows());
  for (size_type i = 0; i < input.rows(); ++i) {
    intSeq row;
    row.length((ULong)input.cols());
    for (size_type j = 0; j < input.cols(); ++j) {
      row[(ULong)j] = input(i, j);
    }
    (*res)[(ULong)i] = row;
  }
  return res;
}

std::vector<int> intSeqToVector(const intSeq& input) {
  std::vector<int> res(input.length());
  for (unsigned int i = 0; i < input.length(); ++i) res[i] = input[i];
  return res;
}

vector3_t floatSeqToVector3(const floatSeq& dofArray) {
  if (dofArray.length() != 3) {
    HPP_THROW(Error, "Expecting vector of size 3, got vector of size "
                         << dofArray.length() << ".");
  }
  // Fill dof vector with dof array.
  vector3_t result;
  for (unsigned int iDof = 0; iDof < 3; ++iDof) {
    result[iDof] = dofArray[iDof];
  }
  return result;
}

vector_t floatSeqToVector(const floatSeq& dofArray,
                          const size_type expectedSize) {
  size_type inputDim = (size_type)dofArray.length();
  if (expectedSize >= 0 && inputDim != expectedSize) {
    HPP_THROW(std::runtime_error,
              "size of input array ("
                  << inputDim << ") is different from the expected size ("
                  << expectedSize << ")");
  }
  // Fill dof vector with dof array.
  vector_t result(inputDim);
  for (size_type iDof = 0; iDof < inputDim; ++iDof) {
    result[iDof] = dofArray[(CORBA::ULong)iDof];
  }
  return result;
}

Configuration_t floatSeqToConfig(const DevicePtr_t& robot,
                                 const floatSeq& dofArray,
                                 bool throwIfNotNormalized) {
  Configuration_t q(floatSeqToVector(dofArray, robot->configSize()));
  if (throwIfNotNormalized) {
    const value_type eps = 1e-8;
    if (!pinocchio::isNormalized(robot, q, eps))
      throw Error(
          "Configuration is not normalized (wrong quaternion or complex "
          "norm).");
  }
  return q;
}

core::matrix_t floatSeqSeqToMatrix(const floatSeqSeq& input,
                                   const size_type expectedRows,
                                   const size_type expectedCols) {
  size_type rows = (size_type)input.length();
  if (expectedRows >= 0 && rows != expectedRows) {
    HPP_THROW(std::runtime_error,
              "number of rows of input floatSeqSeq ("
                  << rows << ") is different from the expected number of rows ("
                  << expectedRows << ")");
  }
  if (rows == 0) return matrix_t(rows, std::max(size_type(0), expectedCols));
  size_type cols = (size_type)input[0].length();
  if (expectedCols >= 0 && cols != expectedCols) {
    HPP_THROW(std::runtime_error,
              "number of cols of input floatSeqSeq ("
                  << cols << ") is different from the expected number of cols ("
                  << expectedCols << ")");
  }
  matrix_t out(rows, cols);
  for (size_type r = 0; r < rows; ++r) {
    const floatSeq& row = input[(CORBA::ULong)r];
    size_type cols_in_row = (size_type)row.length();
    if (cols_in_row != cols)
      HPP_THROW(std::runtime_error,
                "number of cols of input floatSeqSeq ("
                    << cols_in_row
                    << ") is different from the expected number of cols ("
                    << expectedCols << ") in row " << r);
    for (size_type c = 0; c < cols; ++c) {
      out(r, c) = row[(CORBA::ULong)c];
    }
  }
  return out;
}

IntMatrix_t intSeqSeqToMatrix(const intSeqSeq& input,
                              const size_type expectedRows,
                              const size_type expectedCols) {
  size_type rows = (size_type)input.length();
  if (expectedRows >= 0 && rows != expectedRows) {
    HPP_THROW(std::runtime_error,
              "number of rows of input floatSeqSeq ("
                  << rows << ") is different from the expected number of rows ("
                  << expectedRows << ")");
  }
  if (rows == 0) return IntMatrix_t(rows, std::max(size_type(0), expectedCols));
  size_type cols = (size_type)input[0].length();
  if (expectedCols >= 0 && cols != expectedCols) {
    HPP_THROW(std::runtime_error,
              "number of cols of input floatSeqSeq ("
                  << cols << ") is different from the expected number of cols ("
                  << expectedCols << ")");
  }
  IntMatrix_t out(rows, cols);
  for (size_type r = 0; r < rows; ++r) {
    const intSeq& row = input[(CORBA::ULong)r];
    for (size_type c = 0; c < cols; ++c) out(r, c) = row[(CORBA::ULong)c];
  }
  return out;
}

std::vector<bool> boolSeqToVector(const hpp::boolSeq& mask,
                                  unsigned int length) {
  if (mask.length() != length) {
    std::stringstream ss;
    ss << "Mask must be of length " << length;
    throw hpp::Error(ss.str().c_str());
  }
  std::vector<bool> m(length);
  for (size_t i = 0; i < length; i++) m[i] = mask[(CORBA::ULong)i];
  return m;
}

stringSeqSeq* vectorToStringSeqSeq(
    std::vector<std::vector<std::string>> input) {
  stringSeqSeq* seq_ptr = new stringSeqSeq();
  ULong size = (ULong)input.size();
  seq_ptr->length(size);
  for (size_type i = 0; i < size; ++i) {
    Names_t* names = toNames_t<std::vector<std::string>>(input.at(i));
    (*seq_ptr)[(ULong)i] = *names;
  }
  return seq_ptr;
}

constraints::ComparisonTypes_t convertComparison(hpp::ComparisonTypes_t comp) {
  constraints::ComparisonTypes_t res(comp.length());
  for (std::size_t i = 0; i < comp.length(); ++i) {
    switch (comp[(CORBA::ULong)i]) {
      case Equality:
        res[i] = constraints::Equality;
        break;
      case EqualToZero:
        res[i] = constraints::EqualToZero;
        break;
      case Superior:
        res[i] = constraints::Superior;
        break;
      case Inferior:
        res[i] = constraints::Inferior;
        break;
      default:
        abort();
    }
  }
  return res;
}

hpp::ComparisonTypes_t* convertComparison(constraints::ComparisonTypes_t comp) {
  hpp::ComparisonTypes_t* res_ptr = new hpp::ComparisonTypes_t();
  hpp::ComparisonTypes_t& res(*res_ptr);
  res.length((CORBA::ULong)comp.size());
  for (std::size_t i = 0; i < comp.size(); ++i) {
    switch (comp[i]) {
      case constraints::Equality:
        res[(CORBA::ULong)i] = Equality;
        break;
      case constraints::EqualToZero:
        res[(CORBA::ULong)i] = EqualToZero;
        break;
      case constraints::Superior:
        res[(CORBA::ULong)i] = Superior;
        break;
      case constraints::Inferior:
        res[(CORBA::ULong)i] = Inferior;
        break;
      default:
        throw Error("Unknown comparison type");
    }
  }
  return res_ptr;
}

using core::Parameter;

Parameter toParameter(const CORBA::Any& any);

CORBA::Any toCorbaAny(const Parameter& parameter);

struct Parameter_CorbaAny {
 private:
  template <typename first, typename second>
  static inline second as(const first& f) {
    return second(f);
  }

  struct convertToParam {
    const CORBA::Any& corbaAny;
    Parameter& param;
    bool& done;
    void operator()(std::pair<vector_t, floatSeq>) {
      if (done) return;
      floatSeq* d = new floatSeq();
      if (corbaAny >>= d) {
        param = Parameter(floatSeqToVector(*d));
        done = true;
      } else
        delete d;
    }
    void operator()(std::pair<matrix_t, floatSeqSeq>) {
      if (done) return;
      floatSeqSeq* d = new floatSeqSeq();
      if (corbaAny >>= d) {
        param = Parameter(floatSeqSeqToMatrix(*d));
        done = true;
      } else
        delete d;
    }
    template <typename T>
    void operator()(T) {
      if (done) return;
      typename T::second_type d;
      if (corbaAny >>= d) {
        param =
            Parameter(as<typename T::second_type, typename T::first_type>(d));
        done = true;
      }
    }
    convertToParam(const CORBA::Any& a, Parameter& p, bool& d)
        : corbaAny(a), param(p), done(d) {}
  };
  /// TODO: This list of CORBA types is not complete.
  typedef boost::mpl::vector<
      // There is no operator<<= (CORBA::Any, CORBA::Boolean)
      std::pair<bool, CORBA::Boolean>,

      std::pair<size_type, CORBA::LongLong>, std::pair<size_type, CORBA::Long>,
      std::pair<size_type, CORBA::Short>,
      std::pair<size_type, CORBA::ULongLong>,
      std::pair<size_type, CORBA::ULong>, std::pair<size_type, CORBA::UShort>,

      std::pair<value_type, CORBA::Float>, std::pair<value_type, CORBA::Double>,

      std::pair<std::string, const char*>

      // This two do not work because omniidl must be given the option -Wba in
      // order to generate a DynSK.cc file containing the conversion to/from Any
      // type.
      ,
      std::pair<matrix_t, floatSeqSeq>, std::pair<vector_t, floatSeq>>
      Parameter_AnyPairs_t;

 public:
  static Parameter toParameter(const CORBA::Any& an) {
    Parameter out;
    bool done = false;
    convertToParam ret(an, out, done);
    boost::mpl::for_each<Parameter_AnyPairs_t>(ret);
    if (!done) throw hpp::Error("Could not convert to Parameter");
    return out;
  }
  static CORBA::Any toCorbaAny(const Parameter& p) {
    CORBA::Any out;
    switch (p.type()) {
      case Parameter::BOOL:
        out <<= p.boolValue();
        break;
      case Parameter::INT:
        out <<= (CORBA::Long)p.intValue();
        break;
      case Parameter::FLOAT:
        out <<= p.floatValue();
        break;
      case Parameter::STRING:
        out <<= p.stringValue().c_str();
        break;
      case Parameter::VECTOR:
        out <<= vectorToFloatSeq(p.vectorValue());
        break;
      case Parameter::MATRIX:
        out <<= matrixToFloatSeqSeq(p.matrixValue());
        break;
      default:
        throw hpp::Error("Could not convert to CORBA::Any");
        break;
    }
    return out;
  }
};

template <>
inline vector_t Parameter_CorbaAny::as<floatSeq, vector_t>(const floatSeq& in) {
  return floatSeqToVector(in);
}
template <>
inline matrix_t Parameter_CorbaAny::as<floatSeqSeq, matrix_t>(
    const floatSeqSeq& in) {
  return floatSeqSeqToMatrix(in);
}

Parameter toParameter(const CORBA::Any& any) {
  return Parameter_CorbaAny::toParameter(any);
}

CORBA::Any toCorbaAny(const Parameter& parameter) {
  return Parameter_CorbaAny::toCorbaAny(parameter);
}
}  // namespace corbaServer
}  // namespace hpp
