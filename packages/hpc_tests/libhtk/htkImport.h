
#ifndef __HTK_IMPORT_H__
#define __HTK_IMPORT_H__

#include "htkImage.h"

typedef enum en_htkImportKind_t {
  htkImportKind_unknown = -1,
  htkImportKind_raw     = 0x1000,
  htkImportKind_csv,
  htkImportKind_tsv,
  htkImportKind_ppm,
  htkImportKind_text
} htkImportKind_t;

#define htkType_real htkType_float

typedef struct st_htkImportCSV_t {
  int rows;
  int columns;
  void *data;
  htkFile_t file;
  char seperator;
} * htkImportCSV_t;

#define htkImportCSV_getRowCount(csv) ((csv)->rows)
#define htkImportCSV_getColumnCount(csv) ((csv)->columns)
#define htkImportCSV_getData(csv) ((csv)->data)
#define htkImportCSV_getFile(csv) ((csv)->file)
#define htkImportCSV_getSeperator(csv) ((csv)->seperator)

#define htkImportCSV_setRowCount(csv, val)                                 \
  (htkImportCSV_getRowCount(csv) = val)
#define htkImportCSV_setColumnCount(csv, val)                              \
  (htkImportCSV_getColumnCount(csv) = val)
#define htkImportCSV_setData(csv, val) (htkImportCSV_getData(csv) = val)
#define htkImportCSV_setSeperator(csv, val)                                \
  (htkImportCSV_getSeperator(csv) = val)

typedef struct st_htkImportRaw_t {
  int rows;
  int columns;
  void *data;
  htkFile_t file;
} * htkImportRaw_t;

#define htkImportRaw_getRowCount(raw) ((raw)->rows)
#define htkImportRaw_getColumnCount(raw) ((raw)->columns)
#define htkImportRaw_getData(raw) ((raw)->data)
#define htkImportRaw_getFile(raw) ((raw)->file)

#define htkImportRaw_setRowCount(raw, val)                                 \
  (htkImportRaw_getRowCount(raw) = val)
#define htkImportRaw_setColumnCount(raw, val)                              \
  (htkImportRaw_getColumnCount(raw) = val)
#define htkImportRaw_setData(raw, val) (htkImportRaw_getData(raw) = val)

typedef struct st_htkImportText_t {
  int length;
  char *data;
  htkFile_t file;
} * htkImportText_t;

#define htkImportText_getLength(txt) ((txt)->length)
#define htkImportText_getData(txt) ((txt)->data)
#define htkImportText_getFile(txt) ((txt)->file)

#define htkImportText_setLength(txt, val)                                  \
  (htkImportText_getLength(txt) = val)
#define htkImportText_setData(txt, val) (htkImportText_getData(txt) = val)

typedef struct st_htkImport_t {
  htkImportKind_t kind;
  union {
    htkImportRaw_t raw;
    htkImportCSV_t csv;
    htkImportText_t text;
    htkImage_t img;
  } container;
} htkImport_t;

#define htkImport_getKind(imp) ((imp).kind)
#define htkImport_getContainer(imp) ((imp).container)
#define htkImport_getRaw(imp) (htkImport_getContainer(imp).raw)
#define htkImport_getCSV(imp) (htkImport_getContainer(imp).csv)
#define htkImport_getText(imp) (htkImport_getContainer(imp).text)
#define htkImport_getImage(imp) (htkImport_getContainer(imp).img)

#define htkImport_setKind(imp, val) (htkImport_getKind(imp) = val)
#define htkImport_setRaw(imp, val) (htkImport_getRaw(imp) = val)
#define htkImport_setCSV(imp, val) (htkImport_getCSV(imp) = val)
#define htkImport_setText(imp, val) (htkImport_getText(imp) = val)
#define htkImport_setImage(imp, val) (htkImport_getImage(imp) = val)

EXTERN_C void *htkImport(const char *file, int *rows);
void *htkImport(const char *file, int *rows, int *columns);
void *htkImport(const char *file, int *rows, const char *type);
void *htkImport(const char *file, int *resRows, int *resColumns,
               const char *type);
htkImage_t htkImport(const char *file);
int htkImport_flag(const char *file);

#endif /* __HTK_IMPORT_H__ */
