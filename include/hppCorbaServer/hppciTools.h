#ifndef HPPCI_TOOLS_H
#define HPPCI_TOOLS_H

#ifdef PACKAGE_BUGREPORT
#undef PACKAGE_BUGREPORT
#endif
#ifdef PACKAGE_NAME
#undef PACKAGE_NAME
#endif
#ifdef PACKAGE_STRING
#undef PACKAGE_STRING
#endif
#ifdef PACKAGE_TARNAME
#undef PACKAGE_TARNAME
#endif
#ifdef PACKAGE_VERSION
#undef PACKAGE_VERSION
#endif

#include "hppciCommonServer.hh"
#include "KineoUtility/kitMat4.h"

void ConfigurationToCkitMat4(const Configuration inConfig, CkitMat4& outMatrix4);

#endif
