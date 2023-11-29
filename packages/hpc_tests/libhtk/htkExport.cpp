
#include "htk.h"

static inline void htkExportText_setFile(htkExportText_t text,
                                        const char *path) {
  if (text != nullptr) {
    if (htkExportText_getFile(text) != nullptr) {
      htkFile_delete(htkExportText_getFile(text));
    }
    if (path != nullptr) {
      htkExportText_getFile(text) = htkFile_open(path, "w+");
    } else {
      htkExportText_getFile(text) = nullptr;
    }
  }

  return;
}

static inline htkExportText_t htkExportText_new(void) {
  htkExportText_t text;

  text = htkNew(struct st_htkExportText_t);

  htkExportText_getFile(text) = nullptr;
  htkExportText_setLength(text, -1);

  return text;
}

static inline void htkExportText_delete(htkExportText_t text) {
  if (text != nullptr) {
    htkExportText_setFile(text, NULL);
    htkDelete(text);
  }
  return;
}

static inline void htkExportText_write(htkExportText_t text,
                                      const char *data, int length) {
  int ii;
  FILE *handle;
  htkFile_t file;

  if (text == nullptr || htkExportText_getFile(text) == nullptr) {
    return;
  }

  file = htkExportText_getFile(text);

  handle = htkFile_getFileHandle(file);

  if (handle == nullptr) {
    return;
  }

  for (ii = 0; ii < length; ii++) {
    fprintf(handle, "%c", data[ii]);
  }

  return;
}

static inline void htkExportRaw_setFile(htkExportRaw_t raw,
                                       const char *path) {
  if (raw != nullptr) {
    if (htkExportRaw_getFile(raw) != nullptr) {
      htkFile_delete(htkExportRaw_getFile(raw));
    }
    if (path != nullptr) {
      htkExportRaw_getFile(raw) = htkFile_open(path, "w+");
    } else {
      htkExportRaw_getFile(raw) = nullptr;
    }
  }

  return;
}

static inline htkExportRaw_t htkExportRaw_new(void) {
  htkExportRaw_t raw;

  raw = htkNew(struct st_htkExportRaw_t);

  htkExportRaw_getFile(raw) = nullptr;
  htkExportRaw_setRowCount(raw, -1);
  htkExportRaw_setColumnCount(raw, -1);

  return raw;
}

static inline void htkExportRaw_delete(htkExportRaw_t raw) {
  if (raw != nullptr) {
    htkExportRaw_setFile(raw, NULL);
    htkDelete(raw);
  }
  return;
}

static inline void htkExportRaw_write(htkExportRaw_t raw, void *data,
                                     int rows, int columns,
                                     htkType_t type) {
  int ii, jj;
  FILE *handle;
  htkFile_t file;

  if (raw == nullptr || htkExportRaw_getFile(raw) == nullptr) {
    return;
  }

  file = htkExportRaw_getFile(raw);

  handle = htkFile_getFileHandle(file);

  if (handle == nullptr) {
    return;
  }

  if (columns == 1) {
    fprintf(handle, "%d\n", rows);
  } else {
    fprintf(handle, "%d %d\n", rows, columns);
  }

  for (ii = 0; ii < rows; ii++) {
    for (jj = 0; jj < columns; jj++) {
      if (type == htkType_integer) {
        int elem = ((int *)data)[ii * columns + jj];
        fprintf(handle, "%d", elem);
      } else if (type == htkType_ubit8) {
        int elem = ((unsigned char *)data)[ii * columns + jj];
        fprintf(handle, "%d", elem);
      } else {
        htkReal_t elem = ((htkReal_t *)data)[ii * columns + jj];
        fprintf(handle, "%f", elem);
      }
      if (jj == columns - 1) {
        fprintf(handle, "\n");
      } else {
        fprintf(handle, " ");
      }
    }
  }

  return;
}

static inline void htkExportCSV_setFile(htkExportCSV_t csv,
                                       const char *path) {
  if (csv != nullptr) {
    if (htkExportCSV_getFile(csv) != nullptr) {
      htkFile_delete(htkExportCSV_getFile(csv));
    }
    if (path != nullptr) {
      htkExportCSV_getFile(csv) = htkFile_open(path, "w+");
    } else {
      htkExportCSV_getFile(csv) = nullptr;
    }
  }

  return;
}

static inline htkExportCSV_t htkExportCSV_new(void) {
  htkExportCSV_t csv;

  csv = htkNew(struct st_htkExportCSV_t);

  htkExportCSV_getFile(csv) = nullptr;
  htkExportCSV_setColumnCount(csv, -1);
  htkExportCSV_setRowCount(csv, -1);
  htkExportCSV_setSeperator(csv, '\0');

  return csv;
}

static inline void htkExportCSV_delete(htkExportCSV_t csv) {
  if (csv != nullptr) {
    htkExportCSV_setFile(csv, NULL);
    htkDelete(csv);
  }
}

static inline void htkExportCSV_write(htkExportCSV_t csv, void *data,
                                     int rows, int columns, char sep,
                                     htkType_t type) {
  int ii, jj;
  htkFile_t file;
  FILE *handle;
  char seperator[2];

  if (csv == nullptr || htkExportCSV_getFile(csv) == nullptr) {
    return;
  }

  file = htkExportCSV_getFile(csv);

  handle = htkFile_getFileHandle(file);

  if (handle == nullptr) {
    return;
  }

  if (sep == '\0') {
    seperator[0] = ',';
  } else {
    seperator[0] = sep;
  }
  seperator[1] = '\0';

  for (ii = 0; ii < rows; ii++) {
    for (jj = 0; jj < columns; jj++) {
      if (type == htkType_integer) {
        int elem = ((int *)data)[ii * columns + jj];
        fprintf(handle, "%d", elem);
      } else if (type == htkType_ubit8) {
        int elem = ((unsigned char *)data)[ii * columns + jj];
        fprintf(handle, "%d", elem);
      } else {
        htkReal_t elem = ((htkReal_t *)data)[ii * columns + jj];
        fprintf(handle, "%f", elem);
      }
      if (jj == columns - 1) {
        fprintf(handle, "\n");
      } else {
        fprintf(handle, "%s", seperator);
      }
    }
  }

  return;
}

static inline htkExport_t htkExport_open(const char *file,
                                       htkExportKind_t kind) {
  htkExport_t exprt;

  if (file == nullptr) {
    htkLog(ERROR, "Go NULL for file value.");
    htkExit();
  }

  htkExport_setFile(exprt, NULL);
  htkExport_setKind(exprt, kind);

  if (kind == htkExportKind_raw) {
    htkExportRaw_t raw = htkExportRaw_new();
    htkExportRaw_setFile(raw, file);
    htkExport_setRaw(exprt, raw);
  } else if (kind == htkExportKind_text) {
    htkExportText_t txt = htkExportText_new();
    htkExportText_setFile(txt, file);
    htkExport_setText(exprt, txt);
  } else if (kind == htkExportKind_tsv || kind == htkExportKind_csv) {
    htkExportCSV_t csv = htkExportCSV_new();
    if (kind == htkExportKind_csv) {
      htkExportCSV_setSeperator(csv, ',');
    } else {
      htkExportCSV_setSeperator(csv, '\t');
    }
    htkExportCSV_setFile(csv, file);
    htkExport_setCSV(exprt, csv);
  } else if (kind == htkExportKind_ppm) {
    htkExport_setFile(exprt, htkString_duplicate(file));
  } else {
    htkLog(ERROR, "Invalid export type.");
    htkExit();
  }

  return exprt;
}

static inline htkExport_t htkExport_open(const char *file,
                                       const char *type0) {
  htkExport_t exprt;
  htkExportKind_t kind;
  char *type;

  type = htkString_toLower(type0);

  if (htkString_sameQ(type, "csv")) {
    kind = htkExportKind_csv;
  } else if (htkString_sameQ(type, "tsv")) {
    kind = htkExportKind_tsv;
  } else if (htkString_sameQ(type, "raw") || htkString_sameQ(type, "dat")) {
    kind = htkExportKind_raw;
  } else if (htkString_sameQ(type, "ppm") || htkString_sameQ(type, "pbm")) {
    kind = htkExportKind_ppm;
  } else if (htkString_sameQ(type, "txt") || htkString_sameQ(type, "text")) {
    kind = htkExportKind_text;
  } else {
    htkLog(ERROR, "Invalid export type ", type0);
    htkExit();
  }

  exprt = htkExport_open(file, kind);

  htkDelete(type);

  return exprt;
}

static inline void htkExport_close(htkExport_t exprt) {
  htkExportKind_t kind;

  kind = htkExport_getKind(exprt);

  if (htkExport_getFile(exprt)) {
    htkDelete(htkExport_getFile(exprt));
  }

  if (kind == htkExportKind_tsv || kind == htkExportKind_csv) {
    htkExportCSV_t csv = htkExport_getCSV(exprt);
    htkExportCSV_delete(csv);
    htkExport_setCSV(exprt, NULL);
  } else if (kind == htkExportKind_raw) {
    htkExportRaw_t raw = htkExport_getRaw(exprt);
    htkExportRaw_delete(raw);
    htkExport_setRaw(exprt, NULL);
  } else if (kind == htkExportKind_text) {
    htkExportText_t text = htkExport_getText(exprt);
    htkExportText_delete(text);
    htkExport_setText(exprt, NULL);
  } else if (kind == htkExportKind_ppm) {
  } else {
    htkLog(ERROR, "Invalid export type.");
    htkExit();
  }
  return;
}

static inline void htkExport_writeAsImage(htkExport_t exprt, htkImage_t img) {
  htkAssert(htkExport_getKind(exprt) == htkExportKind_ppm);

  htkPPM_export(htkExport_getFile(exprt), img);

  return;
}

static inline void htkExport_write(htkExport_t exprt, void *data, int rows,
                                  int columns, char sep, htkType_t type) {
  htkExportKind_t kind;

  kind = htkExport_getKind(exprt);
  if (kind == htkExportKind_tsv || kind == htkExportKind_csv) {
    htkExportCSV_t csv = htkExport_getCSV(exprt);
    htkExportCSV_write(csv, data, rows, columns, sep, type);
  } else if (kind == htkExportKind_raw) {
    htkExportRaw_t raw = htkExport_getRaw(exprt);
    htkExportRaw_write(raw, data, rows, columns, type);
  } else if (kind == htkExportKind_text) {
    htkExportText_t text = htkExport_getText(exprt);
    if (columns == 0) {
      columns = 1;
    }
    if (rows == 0) {
      rows = 1;
    }
    htkExportText_write(text, (const char *)data, rows * columns);
  } else {
    htkLog(ERROR, "Invalid export type.");
    htkExit();
  }
  return;
}

static inline void htkExport_write(htkExport_t exprt, void *data, int rows,
                                  int columns, htkType_t type) {
  htkExport_write(exprt, data, rows, columns, ',', type);
}

static htkExportKind_t _parseExportExtension(const char *file) {
  char *extension;
  htkExportKind_t kind;

  extension = htkFile_extension(file);

  if (htkString_sameQ(extension, "csv")) {
    kind = htkExportKind_csv;
  } else if (htkString_sameQ(extension, "tsv")) {
    kind = htkExportKind_tsv;
  } else if (htkString_sameQ(extension, "raw") ||
             htkString_sameQ(extension, "dat")) {
    kind = htkExportKind_raw;
  } else if (htkString_sameQ(extension, "text") ||
             htkString_sameQ(extension, "txt")) {
    kind = htkExportKind_text;
  } else if (htkString_sameQ(extension, "ppm") ||
             htkString_sameQ(extension, "pgm") ||
             htkString_sameQ(extension, "pbm")) {
    kind = htkExportKind_ppm;
  } else {
    kind = htkExportKind_unknown;
    htkLog(ERROR, "File ", file, " does not have a compatible extension.");
  }

  htkDelete(extension);

  return kind;
}

static void htkExport(const char *file, void *data, int rows, int columns,
                     htkType_t type) {
  htkExportKind_t kind;
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  kind  = _parseExportExtension(file);
  exprt = htkExport_open(file, kind);

  htkExport_write(exprt, data, rows, columns, type);
  htkExport_close(exprt);
}

void htkExport(const char *file, unsigned char *data, int rows) {
  htkExport(file, data, rows, 1);
  return;
}

void htkExport(const char *file, int *data, int rows) {
  htkExport(file, data, rows, 1);
  return;
}

void htkExport(const char *file, htkReal_t *data, int rows) {
  htkExport(file, data, rows, 1);
  return;
}

void htkExport(const char *file, unsigned char *data, int rows,
              int columns) {
  htkExportKind_t kind;
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  kind  = _parseExportExtension(file);
  exprt = htkExport_open(file, kind);

  htkExport_write(exprt, data, rows, columns, htkType_ubit8);
  htkExport_close(exprt);
}

void htkExport(const char *file, int *data, int rows, int columns) {
  htkExportKind_t kind;
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  kind  = _parseExportExtension(file);
  exprt = htkExport_open(file, kind);

  htkExport_write(exprt, data, rows, columns, htkType_integer);
  htkExport_close(exprt);
}

void htkExport(const char *file, htkReal_t *data, int rows, int columns) {
  htkExportKind_t kind;
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  kind  = _parseExportExtension(file);
  exprt = htkExport_open(file, kind);

  htkExport_write(exprt, data, rows, columns, htkType_real);
  htkExport_close(exprt);
}

void htkExport(const char *file, htkExportKind_t kind, void *data, int rows,
              int columns, htkType_t type) {
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  exprt = htkExport_open(file, kind);

  htkExport_write(exprt, data, rows, columns, type);
  htkExport_close(exprt);
}

void htkExport(const char *file, htkImage_t img) {
  htkExportKind_t kind;
  htkExport_t exprt;

  if (file == nullptr) {
    return;
  }

  kind  = _parseExportExtension(file);
  exprt = htkExport_open(file, kind);

  htkAssert(kind == htkExportKind_ppm);

  htkExport_writeAsImage(exprt, img);
  htkExport_close(exprt);
}

void htkExport_text(const char *file, void *data, int length) {
  htkExport(file, htkExportKind_text, data, 1, length, htkType_ascii);
}
