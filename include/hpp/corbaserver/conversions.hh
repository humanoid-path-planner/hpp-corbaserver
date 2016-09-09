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

# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/common.hh>

namespace hpp {
  namespace corbaServer {
    void toTransform3f (const Transform_ in, Transform3f& out);

    Transform3f toTransform3f (const Transform_ in);

    void toHppTransform (const Transform3f& in, Transform_ out);

    Transform__slice* toHppTransform (const Transform3f& in);

    floatSeq* vectorToFloatseq (const vector_t& input);

    /// Returns a sequence of the rows of the input matrix.
    // Return [ [input.row(0)], [input.row(1)], ...]
    floatSeqSeq* matrixToFloatSeqSeq (const matrix_t& input);

    template <typename InputIt> inline Names_t* toNames_t (InputIt begin, InputIt end)
    {
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
  } // namespace corbaServer
} // namespace hpp

#endif // HPP_CORBASERVER_CONVERSIONS_HH
