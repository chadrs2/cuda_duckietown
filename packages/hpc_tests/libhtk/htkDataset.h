#ifndef __HTK_DATASET_H__
#define __HTK_DATASET_H__

#include "htkImport.h"
#include "htkTypes.h"

typedef struct {
  int rows;
  int cols;
  htkType_t type;
  double minVal;
  double maxVal;
} htkCSV_GenerateParams_t;

typedef struct {
  int rows;
  int cols;
  htkType_t type;
  double minVal;
  double maxVal;
} htkTSV_GenerateParams_t;

typedef struct {
  int rows;
  int cols;
  double minVal;
  double maxVal;
  htkType_t type;
} htkRaw_GenerateParams_t;

typedef struct {
  int width;
  int height;
  int channels;
  double minVal;
  double maxVal;
} htkPPM_GenerateParams_t;

typedef struct { int length; } htkText_GenerateParams_t;

typedef union {
  htkCSV_GenerateParams_t csv;
  htkRaw_GenerateParams_t raw;
  htkTSV_GenerateParams_t tsv;
  htkPPM_GenerateParams_t ppm;
  htkText_GenerateParams_t text;
} htkGenerateParams_t;

EXTERN_C void htkDataset_generate(const char *path, htkExportKind_t kind,
                                 htkGenerateParams_t params);

#endif /* __HTK_DATASET_H__ */
