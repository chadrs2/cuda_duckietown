

#ifndef __HTK_TYPES_H__
#define __HTK_TYPES_H__

#include "htkAssert.h"

typedef bool htkBool;
typedef float htkReal_t;
typedef char htkChar_t;

typedef struct st_htkTimerNode_t *htkTimerNode_t;
typedef struct st_htkTimer_t *htkTimer_t;
typedef struct st_htkLogEntry_t *htkLogEntry_t;
typedef struct st_htkLogger_t *htkLogger_t;
typedef struct st_htkArg_t htkArg_t;
typedef struct st_htkImage_t *htkImage_t;
typedef struct st_htkFile_t *htkFile_t;

#define htkTrue true
#define htkFalse false

typedef enum en_htkType_t {
  htkType_unknown = -1,
  htkType_ascii   = 1,
  htkType_bit8,
  htkType_ubit8,
  htkType_integer,
  htkType_float,
  htkType_double
} htkType_t;

static inline size_t htkType_size(htkType_t ty) {
  switch (ty) {
    case htkType_unknown:
      htkAssert(false && "Invalid htkType_unknown");
      return 0;
    case htkType_ascii:
      return sizeof(char);
    case htkType_bit8:
      return sizeof(char);
    case htkType_ubit8:
      return sizeof(unsigned char);
    case htkType_integer:
      return sizeof(int);
    case htkType_float:
      return sizeof(float);
    case htkType_double:
      return sizeof(double);
  }
  htkAssert(false && "Invalid type");
  return 0;
}

#endif /* __HTK_TYPES_H__ */
