

#include "htk.h"

#define htkFile_maxCount 256

static htkFile_t htkFile_handles[htkFile_maxCount];

static int htkFile_nextIndex(void) {
  int ii;
  for (ii = 0; ii < htkFile_maxCount; ii++) {
    if (htkFile_handles[ii] == nullptr) {
      return ii;
    }
  }
  htkLog(ERROR, "Ran out of file handles.");
  htkExit();
  return -1;
}

htkFile_t htkFile_new(void) {
  int idx       = htkFile_nextIndex();
  htkFile_t file = htkNew(struct st_htkFile_t);

  htkAssert(idx >= 0);

  htkFile_setIndex(file, idx);
  htkFile_setFileName(file, NULL);
  htkFile_setMode(file, NULL);
  htkFile_setFileHandle(file, NULL);
  htkFile_setData(file, NULL);

  htkFile_handles[idx] = file;

  return file;
}

void htkFile_delete(htkFile_t file) {
  if (file != nullptr) {
    int idx = htkFile_getIndex(file);
    if (htkFile_getFileName(file) != nullptr) {
      htkDelete(htkFile_getFileName(file));
    }
    if (htkFile_getMode(file) != nullptr) {
      htkDelete(htkFile_getMode(file));
    }
    if (htkFile_getFileHandle(file) != nullptr) {
      fflush(htkFile_getFileHandle(file));
      fclose(htkFile_getFileHandle(file));
    }
    if (idx >= 0) {
      htkAssert(htkFile_handles[idx] == file);
      htkFile_handles[idx] = nullptr;
    }
    if (htkFile_getData(file) != nullptr) {
      htkDelete(htkFile_getData(file));
    }
    htkDelete(file);
  }
}

void htkFile_init(void) {
  int ii;

  for (ii = 0; ii < htkFile_maxCount; ii++) {
    htkFile_handles[ii] = nullptr;
  }
}

void htkFile_atExit(void) {
  int ii;

  for (ii = 0; ii < htkFile_maxCount; ii++) {
    if (htkFile_handles[ii] != nullptr) {
      htkFile_delete(htkFile_handles[ii]);
    }
  }
}

int htkFile_count(void) {
  int ii, count = 0;

  for (ii = 0; ii < htkFile_maxCount; ii++) {
    if (htkFile_handles[ii] != nullptr) {
      count++;
    }
  }
  return count;
}

htkFile_t htkFile_open(const char *fileName, const char *mode) {
  FILE *handle;
  htkFile_t file;

  if (fileName == nullptr) {
    return NULL;
  }

  handle = fopen(fileName, mode);
  if (handle == nullptr) {
    htkLog(ERROR, "Failed to open ", file, " in mode ", mode);
    return NULL;
  }

  file = htkFile_new();
  htkFile_setFileName(file, htkString_duplicate(fileName));
  htkFile_setMode(file, htkString_duplicate(mode));
  htkFile_setFileHandle(file, handle);

  return file;
}

htkFile_t htkFile_open(const char *fileName) {
  return htkFile_open(fileName, "r");
}

void htkFile_close(htkFile_t file) {
  htkFile_delete(file);
}

char *htkFile_read(htkFile_t file, size_t size, size_t count) {
  size_t res;
  char *buffer;
  size_t bufferLen;
  FILE *handle;

  if (file == nullptr) {
    return NULL;
  }
#ifndef LAZY_FILE_LOAD
  if (htkFile_getData(file) != nullptr) {
    char *data = htkFile_getData(file) + htkFile_getDataOffset(file);
    htkFile_setDataOffset(file, htkFile_getDataOffset(file) + size * count);
    return data;
  }
#endif /* LAZY_FILE_LOAD */

  handle    = htkFile_getFileHandle(file);
  bufferLen = size * count + 1;
  buffer    = htkNewArray(char, bufferLen);

  res = fread(buffer, size, count, handle);
  // make valid C string
  buffer[size * res] = '\0';

  return buffer;
}

char *htkFile_read(htkFile_t file, size_t len) {
  char *buffer = htkFile_read(file, sizeof(char), len);
  return buffer;
}

void htkFile_rewind(htkFile_t file) {
  if (file == nullptr) {
    return;
  }

  if (htkFile_getData(file) == nullptr) {
    FILE *handle;
    handle = htkFile_getFileHandle(file);
    htkAssert(handle != nullptr);
    rewind(handle);
  }
#ifndef LAZY_FILE_LOAD
  else {
    htkFile_setDataOffset(file, 0);
  }
#endif

  return;
}

size_t htkFile_size(htkFile_t file) {
  size_t len;
  FILE *handle;

  if (file == nullptr) {
    return 0;
  }
#ifndef LAZY_FILE_LOAD
  if (htkFile_getData(file) != nullptr) {
    if (htkFile_getLength(file) == 0) {
      htkFile_setLength(file, strlen(htkFile_getData(file)));
    }
    return htkFile_getLength(file);
  }
#endif /* LAZY_FILE_LOAD */

  handle = htkFile_getFileHandle(file);

  fseek(handle, 0, SEEK_END);
  len = ftell(handle);
  rewind(handle);

  return len;
}

char *htkFile_read(htkFile_t file) {
  size_t len;

  if (file == nullptr) {
    return NULL;
  }

  len = htkFile_size(file);

  if (len == 0) {
    return NULL;
  }

  htkFile_setLength(file, len);

  return htkFile_read(file, len);
}

#define MAX_CHARS_PER_LINE (1 << 17)

static char buffer[MAX_CHARS_PER_LINE];

char *htkFile_readLine(htkFile_t file) {
  if (file == nullptr) {
    return NULL;
  }
#ifdef LAZY_FILE_LOAD
  FILE *handle;
  memset(buffer, 0, MAX_CHARS_PER_LINE);

  handle = htkFile_getFileHandle(file);

  if (fgets(buffer, MAX_CHARS_PER_LINE - 1, handle)) {
    return buffer;
  } else {
    // htkLog(ERROR, "Was not able to read line from ",
    // htkFile_getFileName(file));
    return NULL;
  }
#else
  size_t newOffset;
  size_t lenToNewLine = 0;
  const char *tmp;

  if (htkFile_getData(file) == nullptr) {
    htkFile_setData(file, htkFile_read(file));
    fclose(htkFile_getFileHandle(file));
    htkFile_setFileHandle(file, NULL);
    htkFile_setDataOffset(file, 0);
    htkFile_setLength(file, strlen(htkFile_getData(file)));
  }

  memset(buffer, 0, MAX_CHARS_PER_LINE);

  if (htkFile_getDataOffset(file) >= htkFile_getLength(file)) {
    return NULL;
  }

  newOffset = htkFile_getDataOffset(file);
  tmp       = htkFile_getData(file) + htkFile_getDataOffset(file);
  while (newOffset < htkFile_getLength(file) && *tmp != '\n') {
    tmp++;
    lenToNewLine++;
    newOffset++;
  }

  memcpy(buffer, htkFile_getData(file) + htkFile_getDataOffset(file),
         lenToNewLine);
  htkFile_setDataOffset(file, newOffset + 1);

  return buffer;
#endif
}

void htkFile_write(htkFile_t file, const void *buffer, size_t size,
                  size_t count) {
  size_t res;
  FILE *handle;

  if (file == nullptr) {
    return;
  }

  handle = htkFile_getFileHandle(file);

  res = fwrite(buffer, size, count, handle);
  if (res != count) {
    htkLog(ERROR, "Failed to write data to ", htkFile_getFileName(file));
  }

  return;
}

void htkFile_write(htkFile_t file, const void *buffer, size_t len) {
  htkFile_write(file, buffer, sizeof(char), len);
  return;
}

void htkFile_write(htkFile_t file, const char *buffer) {
  size_t len;

  len = strlen(buffer);
  htkFile_write(file, buffer, len);

  return;
}

void htkFile_writeLine(htkFile_t file, const char *buffer0) {
  string buffer = htkString(buffer0, "\n");
  htkFile_write(file, buffer.c_str());
}

void htkFile_write(htkFile_t file, string buffer) {
  htkFile_write(file, buffer.c_str());
}

void htkFile_writeLine(htkFile_t file, string buffer0) {
  string buffer = buffer0 + "\n";
  htkFile_write(file, buffer.c_str());
}

htkBool htkFile_existsQ(const char *path) {
  if (path == nullptr) {
    return htkFalse;
  } else {
    FILE *file = fopen(path, "r");
    if (file != nullptr) {
      fclose(file);
      return htkTrue;
    }
    return htkFalse;
  }
}

char *htkFile_extension(const char *file) {
  char *extension;
  char *extensionLower;
  char *end;
  size_t len;

  len = strlen(file);
  end = (char *)&file[len - 1];
  while (*end != '.') {
    end--;
  }
  if (*end == '.') {
    end++;
  }

  extension      = htkString_duplicate(end);
  extensionLower = htkString_toLower(extension);
  htkDelete(extension);

  return extensionLower;
}
