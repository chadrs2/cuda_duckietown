

#include "htk.h"

static inline void htkImportCSV_setFile(htkImportCSV_t csv,
                                       const char *path) {
  if (csv != nullptr) {
    if (htkImportCSV_getFile(csv) != nullptr) {
      htkFile_delete(htkImportCSV_getFile(csv));
    }
    if (path != nullptr) {
      htkImportCSV_getFile(csv) = htkFile_open(path, "r");
    } else {
      htkImportCSV_getFile(csv) = nullptr;
    }
  }

  return;
}

static inline htkImportCSV_t htkImportCSV_new(void) {
  htkImportCSV_t csv;

  csv = htkNew(struct st_htkImportCSV_t);

  htkImportCSV_setRowCount(csv, -1);
  htkImportCSV_setColumnCount(csv, -1);
  htkImportCSV_setData(csv, NULL);
  htkImportCSV_getFile(csv) = nullptr;
  htkImportCSV_setSeperator(csv, '\0');

  return csv;
}

static inline void htkImportCSV_delete(htkImportCSV_t csv) {
  if (csv != nullptr) {
    htkImportCSV_setFile(csv, NULL);
    if (htkImportCSV_getData(csv)) {
      htkDelete(htkImportCSV_getData(csv));
    }
    htkDelete(csv);
  }
}

static inline htkImportCSV_t htkImportCSV_findDimensions(htkImportCSV_t csv,
                                                       int *resRows,
                                                       int *resColumns) {
  int rows = 0, columns = -1;
  char *line;
  htkFile_t file;
  char seperator[2];

  if (csv == nullptr) {
    return NULL;
  }

  if (htkImportCSV_getSeperator(csv) == '\0') {
    seperator[0] = ',';
  } else {
    seperator[0] = htkImportCSV_getSeperator(csv);
  }
  seperator[1] = '\0';

  file = htkImportCSV_getFile(csv);

  while ((line = htkFile_readLine(file)) != nullptr) {
    int currColumn = 0;
    char *token    = strtok(line, seperator);
    while (token != nullptr) {
      token = strtok(NULL, seperator);
      currColumn++;
    }
    rows++;
    if (columns == -1) {
      columns = currColumn;
    }
    if (columns != currColumn) {
      htkLog(ERROR, "The csv file is not rectangular.");
    }
    htkAssert(columns == currColumn);
  }

  htkFile_rewind(file);

  *resRows    = rows;
  *resColumns = columns;

  return csv;
}

static inline int *csv_readAsInteger(htkFile_t file, char sep, int rows,
                                     int columns) {
  int ii = 0;
  int *data;
  char *line;
  int var;
  char seperator[2];

  if (file == nullptr) {
    return NULL;
  }

  data = htkNewArray(int, rows *columns);

  if (sep == '\0') {
    seperator[0] = ',';
  } else {
    seperator[0] = sep;
  }
  seperator[1] = '\0';

  // printf("cols = %d rows = %d\n", columns, rows);
  if (columns == 1) {
    while ((line = htkFile_readLine(file)) != nullptr) {
      sscanf(line, "%d", &var);
      // printf("reading %d\n", var);
      data[ii++] = var;
    }
  } else {
    while ((line = htkFile_readLine(file)) != nullptr) {
      char *token = strtok(line, seperator);
      while (token != nullptr) {
        sscanf(token, "%d", &var);
        token      = strtok(NULL, seperator);
        data[ii++] = var;
      }
    }
  }

  return data;
}

static inline htkReal_t *csv_readAsReal(htkFile_t file, char sep, int rows,
                                       int columns) {
  int ii = 0;
  htkReal_t *data;
  char *line;
  htkReal_t var;
  char seperator[2];

  if (file == nullptr) {
    return NULL;
  }

  data = htkNewArray(htkReal_t, rows * columns);

  if (sep == '\0') {
    seperator[0] = ',';
  } else {
    seperator[0] = sep;
  }
  seperator[1] = '\0';

  if (columns == 1) {
    while ((line = htkFile_readLine(file)) != nullptr) {
      sscanf(line, "%f", &var);
      data[ii++] = var;
    }
  } else {
    while ((line = htkFile_readLine(file)) != nullptr) {
      char *token = strtok(line, seperator);
      while (token != nullptr) {
        sscanf(token, "%f", &var);
        token      = strtok(NULL, seperator);
        data[ii++] = var;
      }
    }
  }

  return data;
}

static inline htkImportCSV_t htkImportCSV_read(htkImportCSV_t csv,
                                             htkType_t type) {
  void *data;
  htkFile_t file;
  char seperator;
  int rows, columns;

  if (csv == nullptr) {
    return NULL;
  }

  if (htkImportCSV_getRowCount(csv) == -1 ||
      htkImportCSV_getColumnCount(csv) == -1) {
    if (htkImportCSV_findDimensions(csv, &rows, &columns) == nullptr) {
      htkLog(ERROR, "Failed to figure out csv dimensions.");
      return NULL;
    }
    htkImportCSV_setRowCount(csv, rows);
    htkImportCSV_setColumnCount(csv, columns);
  }

  file      = htkImportCSV_getFile(csv);
  seperator = htkImportCSV_getSeperator(csv);
  rows      = htkImportCSV_getRowCount(csv);
  columns   = htkImportCSV_getColumnCount(csv);

  if (htkImportCSV_getData(csv) != nullptr) {
    htkDelete(htkImportCSV_getData(csv));
    htkImportCSV_setData(csv, NULL);
  }

  if (type == htkType_integer) {
    // printf("ReadXXXing as integer...\n");
    data = csv_readAsInteger(file, seperator, rows, columns);
  } else {
    data = csv_readAsReal(file, seperator, rows, columns);
  }

  htkImportCSV_setData(csv, data);

  return csv;
}

static inline htkImportCSV_t htkImportCSV_readAsInteger(htkImportCSV_t csv) {
  return htkImportCSV_read(csv, htkType_integer);
}

static inline htkImportCSV_t htkImportCSV_readAsReal(htkImportCSV_t csv) {
  return htkImportCSV_read(csv, htkType_real);
}

static inline void htkImportRaw_setFile(htkImportRaw_t raw,
                                       const char *path) {
  if (raw != nullptr) {
    if (htkImportRaw_getFile(raw) != nullptr) {
      htkFile_delete(htkImportRaw_getFile(raw));
    }
    if (path != nullptr) {
      htkImportRaw_getFile(raw) = htkFile_open(path, "r");
    } else {
      htkImportRaw_getFile(raw) = nullptr;
    }
  }

  return;
}

static inline htkImportRaw_t htkImportRaw_new(void) {
  htkImportRaw_t raw;

  raw = htkNew(struct st_htkImportRaw_t);

  htkImportRaw_setRowCount(raw, -1);
  htkImportRaw_setColumnCount(raw, -1);
  htkImportRaw_setData(raw, NULL);
  htkImportRaw_getFile(raw) = nullptr;

  return raw;
}

static inline void htkImportRaw_delete(htkImportRaw_t raw) {
  if (raw != nullptr) {
    htkImportRaw_setFile(raw, NULL);
    if (htkImportRaw_getData(raw)) {
      htkDelete(htkImportRaw_getData(raw));
    }
    htkDelete(raw);
  }
}

static inline htkBool lineHasSpace(const char *line) {
  while (*line != '\0') {
    if (*line == ' ') {
      return htkTrue;
    }
    line++;
  }
  return htkFalse;
}

static inline char *lineStrip(const char *line) {
  char *sl    = htkString_duplicate(line);
  char *iter  = sl;
  size_t slen = strlen(line);

  iter += slen - 1;
  while (*iter == '\0' || *iter == '\r' || *iter == '\t' ||
         *iter == '\n' || *iter == ' ') {
    *iter-- = '\0';
  }
  return sl;
}

static inline htkBool htkImportRaw_findDimensions(htkImportRaw_t raw) {
  if (raw != nullptr) {
    int rows;
    int columns;
    char *line;
    htkFile_t file;
    char *strippedLine;

    file = htkImportRaw_getFile(raw);

    htkFile_rewind(file);

    line = htkFile_readLine(file);

    if (line == nullptr) {
      return htkTrue;
    }

    strippedLine = lineStrip(line);

    if (lineHasSpace(strippedLine)) {
      sscanf(strippedLine, "%d %d", &rows, &columns);
    } else {
      columns = 1;
      sscanf(strippedLine, "%d", &rows);
    }

    htkImportRaw_setRowCount(raw, rows);
    htkImportRaw_setColumnCount(raw, columns);

    htkDelete(strippedLine);

    return htkFalse;
  }

  return htkTrue;
}

static inline htkImportRaw_t htkImportRaw_read(htkImportRaw_t raw,
                                             htkType_t type) {
  void *data;
  htkFile_t file;
  char seperator;
  int rows, columns;

  if (raw == nullptr) {
    return NULL;
  }

  if (htkImportRaw_getRowCount(raw) == -1 ||
      htkImportRaw_getColumnCount(raw) == -1) {
    if (htkImportRaw_findDimensions(raw)) {
      htkLog(ERROR, "Failed to figure out raw dimensions.");
      return NULL;
    }
  }

  file      = htkImportRaw_getFile(raw);
  seperator = ' ';
  rows      = htkImportRaw_getRowCount(raw);
  columns   = htkImportRaw_getColumnCount(raw);

  if (htkImportRaw_getData(raw) != nullptr) {
    htkDelete(htkImportRaw_getData(raw));
    htkImportRaw_setData(raw, NULL);
  }

  if (type == htkType_integer) {
    // printf("Rdin gas integer...\n");
    data = csv_readAsInteger(file, seperator, rows, columns);
  } else {
    data = csv_readAsReal(file, seperator, rows, columns);
  }

  htkImportRaw_setData(raw, data);

  return raw;
}

static inline htkImportRaw_t htkImportRaw_readAsInteger(htkImportRaw_t raw) {
  return htkImportRaw_read(raw, htkType_integer);
}

static inline htkImportRaw_t htkImportRaw_readAsReal(htkImportRaw_t raw) {
  return htkImportRaw_read(raw, htkType_real);
}

static inline htkImportText_t htkImportText_new(void) {
  htkImportText_t text;

  text = htkNew(struct st_htkImportText_t);

  htkImportText_setLength(text, 0);
  htkImportText_setData(text, NULL);
  htkImportText_getFile(text) = nullptr;

  return text;
}

static inline void htkImportText_setFile(htkImportText_t text,
                                        const char *path) {
  if (text != nullptr) {
    if (htkImportText_getFile(text) != nullptr) {
      htkFile_delete(htkImportText_getFile(text));
    }
    if (path != nullptr) {
      htkImportText_getFile(text) = htkFile_open(path, "r");
    } else {
      htkImportText_getFile(text) = nullptr;
    }
  }

  return;
}

static inline void htkImportText_delete(htkImportText_t text) {
  if (text != nullptr) {
    htkImportText_setFile(text, NULL);
    if (htkImportText_getData(text)) {
      htkDelete(htkImportText_getData(text));
    }
    htkDelete(text);
  }
}

static inline htkImportText_t htkImportText_read(htkImportText_t text) {
  char *data;
  htkFile_t file;
  int length;

  if (text == nullptr) {
    return NULL;
  }

  file = htkImportText_getFile(text);

  if (htkImportText_getData(text) != nullptr) {
    htkDelete(htkImportText_getData(text));
    htkImportText_setData(text, NULL);
  }

  length = htkFile_size(file);
  data   = htkFile_read(file, length);

  htkImportText_setData(text, data);
  htkImportText_setLength(text, length);

  return text;
}

static inline htkImport_t htkImport_open(const char *file,
                                       htkImportKind_t kind) {
  htkImport_t imp;

  if (file == nullptr) {
    htkLog(ERROR, "Go NULL for file value.");
    htkExit();
  }

  if (!htkFile_existsQ(file)) {
    htkLog(ERROR, "File ", file, " does not exist.");
    htkExit();
  }

  htkImport_setKind(imp, kind);

  if (kind == htkImportKind_raw) {
    htkImportRaw_t raw = htkImportRaw_new();
    htkImportRaw_setFile(raw, file);
    htkImport_setRaw(imp, raw);
  } else if (kind == htkImportKind_tsv || kind == htkImportKind_csv) {
    htkImportCSV_t csv = htkImportCSV_new();
    if (kind == htkImportKind_csv) {
      htkImportCSV_setSeperator(csv, ',');
    } else {
      htkImportCSV_setSeperator(csv, '\t');
    }
    htkImportCSV_setFile(csv, file);
    htkImport_setCSV(imp, csv);
  } else if (kind == htkImportKind_text) {
    htkImportText_t text = htkImportText_new();
    htkImportText_setFile(text, file);
    htkImport_setText(imp, text);
  } else if (kind == htkImportKind_ppm) {
    htkImage_t img = htkPPM_import(file);
    htkImport_setImage(imp, img);
  } else {
    htkLog(ERROR, "Invalid import type.");
    htkExit();
  }

  return imp;
}

static inline htkImport_t htkImport_open(const char *file,
                                       const char *type0) {
  htkImport_t imp;
  htkImportKind_t kind;
  char *type;

  type = htkString_toLower(type0);

  if (htkString_sameQ(type, "csv")) {
    kind = htkImportKind_csv;
  } else if (htkString_sameQ(type, "tsv")) {
    kind = htkImportKind_tsv;
  } else if (htkString_sameQ(type, "raw") || htkString_sameQ(type, "dat")) {
    kind = htkImportKind_raw;
  } else if (htkString_sameQ(type, "ppm")) {
    kind = htkImportKind_ppm;
  } else if (htkString_sameQ(type, "text") || htkString_sameQ(type, "txt")) {
    kind = htkImportKind_text;
  } else {
    htkLog(ERROR, "Invalid import type ", type0);
    htkExit();
  }

  imp = htkImport_open(file, kind);

  htkDelete(type);

  return imp;
}

static inline void htkImport_close(htkImport_t imp) {
  htkImportKind_t kind;

  kind = htkImport_getKind(imp);
  if (kind == htkImportKind_tsv || kind == htkImportKind_csv) {
    htkImportCSV_t csv = htkImport_getCSV(imp);
    htkImportCSV_delete(csv);
    htkImport_setCSV(imp, NULL);
  } else if (kind == htkImportKind_raw) {
    htkImportRaw_t raw = htkImport_getRaw(imp);
    htkImportRaw_delete(raw);
    htkImport_setRaw(imp, NULL);
  } else if (kind == htkImportKind_text) {
    htkImportText_t text = htkImport_getText(imp);
    htkImportText_delete(text);
    htkImport_setText(imp, NULL);
  } else if (kind == htkImportKind_ppm) {
  } else {
    htkLog(ERROR, "Invalid import type.");
    htkExit();
  }
  return;
}

static inline void *htkImport_read(htkImport_t imp, htkType_t type) {
  void *data = nullptr;
  htkImportKind_t kind;

  kind = htkImport_getKind(imp);
  if (kind == htkImportKind_tsv || kind == htkImportKind_csv) {
    htkImportCSV_t csv = htkImport_getCSV(imp);
    htkImportCSV_read(csv, type);
    data = htkImportCSV_getData(csv);
  } else if (kind == htkImportKind_raw) {
    htkImportRaw_t raw = htkImport_getRaw(imp);
    htkImportRaw_read(raw, type);
    data = htkImportRaw_getData(raw);
  } else if (htkImportKind_text == kind) {
    htkImportText_t text = htkImport_getText(imp);
    text                = htkImportText_read(text);
    data                = htkImportText_getData(text);

  } else {
    htkLog(ERROR, "Invalid import type.");
    htkExit();
  }
  return data;
}

static inline int *htkImport_readAsInteger(htkImport_t imp) {
  void *data = htkImport_read(imp, htkType_integer);
  return (int *)data;
}

static inline htkReal_t *htkImport_readAsReal(htkImport_t imp) {
  void *data = htkImport_read(imp, htkType_real);
  return (htkReal_t *)data;
}

static inline htkChar_t *htkImport_readAsText(htkImport_t imp) {
  void *data = htkImport_read(imp, htkType_ubit8);
  return (htkChar_t *)data;
}

static htkImportKind_t _parseImportExtension(const char *file) {
  char *extension;
  htkImportKind_t kind;

  extension = htkFile_extension(file);

  if (htkString_sameQ(extension, "csv")) {
    kind = htkImportKind_csv;
  } else if (htkString_sameQ(extension, "tsv")) {
    kind = htkImportKind_tsv;
  } else if (htkString_sameQ(extension, "raw") ||
             htkString_sameQ(extension, "dat")) {
    kind = htkImportKind_raw;
  } else if (htkString_sameQ(extension, "ppm") ||
             htkString_sameQ(extension, "pgm") ||
             htkString_sameQ(extension, "pbm")) {
    kind = htkImportKind_ppm;
  } else if (htkString_sameQ(extension, "text") ||
             htkString_sameQ(extension, "txt")) {
    kind = htkImportKind_text;
  } else {
    kind = htkImportKind_unknown;
    htkLog(ERROR, "File ", file, " does not have a compatible extension.");
  }

  htkDelete(extension);

  return kind;
}

void *htkImport(const char *file, int *resRows, int *resColumns,
               const char *type) {
  void *data, *res;
  htkImport_t imp;
  size_t sz;
  int columns = 0, rows = 0;
  htkImportKind_t kind;

  if (file == nullptr) {
    fprintf(stderr, "Failed to import file.\n");
    htkExit();
  }

  kind = _parseImportExtension(file);

  htkAssert(kind != htkImportKind_unknown);

  imp = htkImport_open(file, kind);
  if (htkString_sameQ(type, "Real")) {
    data = htkImport_readAsReal(imp);
    sz   = sizeof(htkReal_t);
  } else if (htkString_sameQ(type, "Text")) {
    data = htkImport_readAsText(imp);
    sz   = sizeof(char);
  } else {
    // printf("Reading as integer..d\n");
    data = htkImport_readAsInteger(imp);
    sz   = sizeof(int);
  }

  if (kind == htkImportKind_csv || kind == htkImportKind_tsv) {
    rows    = htkImportCSV_getRowCount(htkImport_getCSV(imp));
    columns = htkImportCSV_getColumnCount(htkImport_getCSV(imp));
  } else if (kind == htkImportKind_raw) {
    rows    = htkImportRaw_getRowCount(htkImport_getRaw(imp));
    columns = htkImportRaw_getColumnCount(htkImport_getRaw(imp));
  } else if (kind == htkImportKind_text) {
    rows    = 1;
    columns = htkImportText_getLength(htkImport_getText(imp));
  }

  if (rows == 1 && columns > 0) {
    rows    = columns;
    columns = 1;
  }

  if (resRows != nullptr) {
    *resRows = rows;
  }

  if (resColumns != nullptr) {
    *resColumns = columns;
  }

  res = htkMalloc(sz * rows * columns);
  memcpy(res, data, sz * rows * columns);

  htkImport_close(imp);

  return res;
}

void *htkImport(const char *file, int *rows, int *columns) {
  return htkImport(file, rows, columns, "Real");
}

EXTERN_C void *htkImport(const char *file, int *rows) {
  return htkImport(file, rows, NULL, "Real");
}

void *htkImport(const char *file, int *res_rows, const char *type) {
  int cols, rows;
  void *res = htkImport(file, &rows, &cols, type);
  if (rows == 1 && cols > 1) {
    rows = cols;
  }
  *res_rows = rows;
  return res;
}

htkImage_t htkImport(const char *file) {
  htkImage_t img;
  htkImport_t imp;
  htkImportKind_t kind;

  if (file == nullptr) {
    fprintf(stderr, "Failed to import file.\n");
    htkExit();
  }

  kind = _parseImportExtension(file);

  htkAssert(kind == htkImportKind_ppm);

  imp = htkImport_open(file, kind);
  img = htkImport_getImage(imp);
  htkImport_close(imp);

  return img;
}

int htkImport_flag(const char *file) {
  int res;
  htkFile_t fh      = htkFile_open(file, "r");
  const char *line = htkFile_readLine(fh);
  sscanf(line, "%d", &res);
  htkFile_close(fh);
  return res;
}
