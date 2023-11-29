

#ifndef __HTK_ASSERT_H__
#define __HTK_ASSERT_H__

#include <assert.h>

#ifdef HTK_DEBUG
#define htkAssert(cond) assert(cond)
#define htkAssertMessage(msg, cond)                                        \
  do {                                                                    \
    if (!(cond)) {                                                        \
      htkPrint(msg);                                                       \
      htkAssert(cond);                                                     \
    }                                                                     \
  } while (0)
#else /* HTK_DEBUG */
#define htkAssert(...)
#define htkAssertMessage(...)
#endif /* HTK_DEBUG */

#define htkTodo(msg) htkAssertMessage(msg, false)

#endif /* __HTK_ASSERT_H__ */
