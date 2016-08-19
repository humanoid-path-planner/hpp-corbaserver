// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPPCORBASERVER_TOOLS_HH
# define HPPCORBASERVER_TOOLS_HH

# include <hpp/corbaserver/config.hh>
# include <hpp/corbaserver/fwd.hh>
# include "hpp/corbaserver/common.hh"

# include "hpp/core/problem-solver.hh"

HPP_CORBASERVER_LOCAL void
hppTransformToTransform3f (const CORBA::Double* inConfig,
			    hpp::corbaServer::Transform3f& outMatrix4);

HPP_CORBASERVER_LOCAL void
Transform3fTohppTransform (const hpp::corbaServer::Transform3f& transform,
			   CORBA::Double* config);

HPP_CORBASERVER_LOCAL hpp::floatSeq* vectorToFloatseq
(const hpp::core::vector_t& input);

HPP_CORBASERVER_LOCAL hpp::floatSeqSeq* matrixToFloatSeqSeq
(const hpp::core::matrix_t& input);

template <typename InputIt>
inline hpp::Names_t* toNames_t (InputIt begin, InputIt end) {
  using hpp::Names_t;
  std::size_t len = std::distance (begin, end);
  char** nameList = Names_t::allocbuf((CORBA::ULong) len);
  Names_t *ret = new Names_t ((CORBA::ULong) len, (CORBA::ULong) len, nameList);

  std::size_t i = 0;
  while (begin != end) {
    nameList[i] = new char[begin->length ()+1];
    strcpy (nameList[i], begin->c_str ());
    ++begin;
    ++i;
  }
  return ret;
}

hpp::corbaServer::DevicePtr_t getRobotOrThrow (hpp::core::ProblemSolverPtr_t p);

// ----------------------------------------
// BuildException
// ----------------------------------------

struct BuildExceptionEnd {};

template <typename exception> struct BuildException;

namespace internal {
  template <typename exception, typename In>
    struct BuildExceptionOut {
      typedef BuildException<exception>& type;

      static type run(BuildException<exception>& be, const In& t) { be.ss << t; return be; }
    };
}

template <typename exception>
struct BuildException
{
  std::stringstream ss;

  template <typename T> typename internal::BuildExceptionOut<exception, T>::type operator<< (const T& t) {
    return internal::BuildExceptionOut<exception, T>::run (*this, t);
  }

  void raise () {
    exception exc = internal::BuildExceptionOut<exception, BuildExceptionEnd>::
      run(*this, BuildExceptionEnd());
    throw exc;
  }
};

// ----------------------------------------
// BuildException - template specialization
// ----------------------------------------
namespace internal {
  template <typename exception>
    struct BuildExceptionOut<exception, BuildExceptionEnd> {
      typedef exception type;

      static type run(BuildException<exception>& be, const BuildExceptionEnd&) { return exception(be.ss.str()); }
    };

  template <>
    struct BuildExceptionOut<hpp::Error, BuildExceptionEnd> {
      typedef hpp::Error type;

      static type run(BuildException<hpp::Error>& be, const BuildExceptionEnd&) { return hpp::Error(be.ss.str().c_str()); }
    };
}

#endif //! HPPCORBASERVER_TOOLS_HH
