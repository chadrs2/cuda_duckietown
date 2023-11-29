

#ifndef __HTK_EXPORT_H__
#define __HTK_EXPORT_H__

#include "htk.h"
#include "htkFile.h"
#include "htkPPM.h"

typedef enum en_htkExportKind_t {
  htkExportKind_unknown = -1,
  htkExportKind_raw     = 0x1000,
  htkExportKind_csv,
  htkExportKind_tsv,
  htkExportKind_ppm,
  htkExportKind_text,
} htkExportKind_t;

typedef struct st_htkExportText_t {
  int length;
  htkFile_t file;
} * htkExportText_t;

#define htkExportText_getLength(txt) ((txt)->length)
#define htkExportText_getFile(txt) ((txt)->file)

#define htkExportText_setLength(txt, val)                                  \
  (htkExportText_getLength(txt) = val)

typedef struct st_htkExportRaw_t {
  int rows;
  int columns;
  htkFile_t file;
} * htkExportRaw_t;

#define htkExportRaw_getColumnCount(raw) ((raw)->columns)
#define htkExportRaw_getRowCount(raw) ((raw)->rows)
#define htkExportRaw_getFile(raw) ((raw)->file)

#define htkExportRaw_setRowCount(raw, val)                                 \
  (htkExportRaw_getRowCount(raw) = val)
#define htkExportRaw_setColumnCount(raw, val)                              \
  (htkExportRaw_getColumnCount(raw) = val)

typedef struct st_htkExportCSV_t {
  int rows;
  int columns;
  htkFile_t file;
  char seperator;
} * htkExportCSV_t;

#define htkExportCSV_getRowCount(csv) ((csv)->rows)
#define htkExportCSV_getColumnCount(csv) ((csv)->columns)
#define htkExportCSV_getFile(csv) ((csv)->file)
#define htkExportCSV_getSeperator(csv) ((csv)->seperator)

#define htkExportCSV_setRowCount(csv, val)                                 \
  (htkExportCSV_getRowCount(csv) = val)
#define htkExportCSV_setColumnCount(csv, val)                              \
  (htkExportCSV_getColumnCount(csv) = val)
#define htkExportCSV_setSeperator(csv, val)                                \
  (htkExportCSV_getSeperator(csv) = val)

typedef struct st_htkExport_t {
  htkExportKind_t kind;
  union {
    htkExportRaw_t raw;
    htkExportCSV_t csv;
    htkImage_t img;
    htkExportText_t text;
  } container;
  char *file;
} htkExport_t;

#define htkExport_getKind(exprt) ((exprt).kind)
#define htkExport_getContainer(exprt) ((exprt).container)
#define htkExport_getRaw(exprt) (htkExport_getContainer(exprt).raw)
#define htkExport_getCSV(exprt) (htkExport_getContainer(exprt).csv)
#define htkExport_getImage(exprt) (htkExport_getContainer(exprt).img)
#define htkExport_getText(exprt) (htkExport_getContainer(exprt).text)
#define htkExport_getFile(exprt) ((exprt).file)

#define htkExport_setKind(exprt, val) (htkExport_getKind(exprt) = val)
#define htkExport_setRaw(exprt, val) (htkExport_getRaw(exprt) = val)
#define htkExport_setCSV(exprt, val) (htkExport_getCSV(exprt) = val)
#define htkExport_setImage(exprt, val) (htkExport_getImage(exprt) = val)
#define htkExport_setText(exprt, val) (htkExport_getText(exprt) = val)
#define htkExport_setFile(exprt, val) (htkExport_getFile(exprt) = val)

void htkExport(const char *file, int *data, int rows, int columns);
void htkExport(const char *file, int *data, int rows);
void htkExport(const char *file, unsigned char *data, int rows,
              int columns);
void htkExport(const char *file, unsigned char *data, int rows);
void htkExport(const char *file, int *data, int rows, int columns);
void htkExport(const char *file, int *data, int rows);
void htkExport(const char *file, htkReal_t *data, int rows, int columns);
void htkExport(const char *file, htkReal_t *data, int rows);
void htkExport(const char *file, htkImage_t img);

void htkExport(const char *file, htkExportKind_t kind, void *data, int rows,
              int columns, htkType_t type);

void htkExport_text(const char *file, void *data, int length);

#endif /* __HTK_EXPORT_H__ */
