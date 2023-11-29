#include "htk.h"

template <typename T>
static inline T _min(const T &x, const T &y) {
  return x < y ? x : y;
}

template <typename T>
static inline T _max(const T &x, const T &y) {
  return x > y ? x : y;
}

template <typename T>
inline T lerp(const double &x, const T &start, const T &end) {
  return (1 - x) * start + x * end;
}

static inline void genRandom(void *trgt, htkType_t type, double minVal,
                             double maxVal) {
  const int span  = maxVal - minVal;
  const int r     = rand();
  const double rf = ((double)r) / ((double)RAND_MAX);
  switch (type) {
    case htkType_ascii:
      *((char *)trgt) = (r % span) + minVal; // random printable character;
      break;
    case htkType_bit8:
      *((char *)trgt) = lerp<char>(rf, minVal, maxVal);
      break;
    case htkType_ubit8:
      *((unsigned char *)trgt) = lerp<unsigned char>(rf, minVal, maxVal);
      break;
    case htkType_integer:
      *((int *)trgt) = lerp<int>(rf, minVal, maxVal);
      break;
    case htkType_float: {
      *((float *)trgt) = lerp<float>(rf, minVal, maxVal);
      break;
    }
    case htkType_double: {
      *((double *)trgt) = lerp<double>(rf, minVal, maxVal);
      break;
    }
    case htkType_unknown:
      htkAssert(false && "Invalid htkType_unknown");
      break;
  }
  return;
}

static inline void *genRandomList(htkType_t type, size_t len, double minVal,
                                  double maxVal) {
  size_t ii;
  void *data = htkNewArray(char, htkType_size(type) * len);
  switch (type) {
    case htkType_ascii:
    case htkType_bit8: {
      char *iter = (char *)data;
      for (ii = 0; ii < len; ii++) {
        genRandom(iter++, type, minVal, maxVal);
      }
      break;
    }
    case htkType_ubit8: {
      unsigned char *iter = (unsigned char *)data;
      for (ii = 0; ii < len; ii++) {
        genRandom(iter++, type, minVal, maxVal);
      }
      break;
    }
    case htkType_integer: {
      int *iter = (int *)data;
      for (ii = 0; ii < len; ii++) {
        genRandom(iter++, type, minVal, maxVal);
      }
      break;
    }
    case htkType_float: {
      float *iter = (float *)data;
      for (ii = 0; ii < len; ii++) {
        genRandom(iter++, type, minVal, maxVal);
      }
      break;
    }
    case htkType_double: {
      double *iter = (double *)data;
      for (ii = 0; ii < len; ii++) {
        genRandom(iter++, type, minVal, maxVal);
      }
      break;
    }
    case htkType_unknown:
      htkAssert(false && "Invalid htkType_unknown");
      break;
  }
  return data;
}

static void genRaw(const char *path, htkRaw_GenerateParams_t params) {
  int rows      = _max(1, params.rows);
  int cols      = _max(1, params.cols);
  double minVal = params.minVal;
  double maxVal = params.maxVal;
  htkType_t type = params.type;
  void *data    = genRandomList(type, rows * cols, minVal, maxVal);
  htkExport(path, htkExportKind_raw, data, rows, cols, type);
  htkDelete(data);
}

static void genCSV(const char *path, htkCSV_GenerateParams_t params) {
  int rows      = _max(1, params.rows);
  int cols      = _max(1, params.cols);
  double minVal = params.minVal;
  double maxVal = params.maxVal;
  htkType_t type = params.type;
  void *data    = genRandomList(type, rows * cols, minVal, maxVal);
  htkExport(path, htkExportKind_csv, data, rows, cols, type);
  htkDelete(data);
}

static void genTSV(const char *path, htkTSV_GenerateParams_t params) {
  int rows      = _max(1, params.rows);
  int cols      = _max(1, params.cols);
  double minVal = params.minVal;
  double maxVal = params.maxVal;
  htkType_t type = params.type;
  void *data    = genRandomList(type, rows * cols, minVal, maxVal);
  htkExport(path, htkExportKind_tsv, data, rows, cols, type);
  htkDelete(data);
}

static void genText(const char *path, htkText_GenerateParams_t params) {
  int length    = _max(1, params.length);
  htkType_t type = htkType_ascii;
  void *data    = genRandomList(type, length, 32, 128);
  htkExport(path, htkExportKind_text, data, length, 1, type);
  htkDelete(data);
}

static void genPPM(const char *path, htkPPM_GenerateParams_t params) {
  int width     = _max(1, params.width);
  int height    = _max(1, params.height);
  int channels  = _max(1, params.channels);
  double minVal = params.minVal;
  double maxVal = params.maxVal;
  htkType_t type = htkType_float;
  float *data   = (float *)genRandomList(type, width * height * channels,
                                       minVal, maxVal);
  htkImage_t img = htkImage_new(width, height, channels, data);
  htkExport(path, img);
  htkImage_delete(img);
}

EXTERN_C void htkDataset_generate(const char *path, htkExportKind_t kind,
                                 htkGenerateParams_t params) {
  htkDirectory_create(htkDirectory_name(path));

  switch (kind) {
    case htkExportKind_raw:
      genRaw(path, params.raw);
      break;
    case htkExportKind_csv:
      genCSV(path, params.csv);
      break;
    case htkExportKind_tsv:
      genTSV(path, params.tsv);
      break;
    case htkExportKind_ppm:
      genPPM(path, params.ppm);
      break;
    case htkExportKind_text:
      genText(path, params.text);
      break;
    default:
      htkAssert(false && "Invalid Export kind");
  }
}
