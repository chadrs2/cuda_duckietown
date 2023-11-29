#ifndef __HTK_UTILS_H__
#define __HTK_UTILS_H__

#include "htkString.h"
#include "vendor/sole.hpp"

#ifdef HTK_DEBUG
#define DEBUG(...) __VA_ARGS__
#else /* HTK_DEBUG */
#define DEBUG(...)
#endif /* HTK_DEBUG */

static char* uuid() {
  auto u4 = sole::uuid4();
  return htkString_duplicate(u4.str());
}

#endif /* __HTK_UTILS_H__ */
