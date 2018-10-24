#pragma once
#ifndef QI_OBJECTUID_HPP
#define QI_OBJECTUID_HPP

#include <qi/ptruid.hpp>

namespace qi
{
  /// Unique identifier of an object being referred to by a qi::Object instance.
  ///
  /// Note for users: your code SHALL NOT assume that ObjectUid will always
  /// be implemented as an alias to PtrUid.
  /// The definition of ObjectUid may be changed in the future
  /// (while still remaining convertible to PtrUid).
  using ObjectUid = PtrUid;
}

#endif
