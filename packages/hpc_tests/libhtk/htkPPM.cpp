
#include <math.h>
#include "htk.h"

static inline float _min(float x, float y) {
  return x < y ? x : y;
}

static inline float _max(float x, float y) {
  return x > y ? x : y;
}

static inline float _clamp(float x, float start, float end) {
  return _min(_max(x, start), end);
}

static const char *skipSpaces(const char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
    if (*line == '\0') {
      break;
    }
  }
  return line;
}

static char nextNonSpaceChar(const char *line0) {
  const char *line = skipSpaces(line0);
  return *line;
}

static htkBool isComment(const char *line) {
  char nextChar = nextNonSpaceChar(line);
  if (nextChar == '\0') {
    return htkTrue;
  } else {
    return nextChar == '#';
  }
}

static void parseDimensions(const char *line0, int *width, int *height) {
  const char *line = skipSpaces(line0);
  sscanf(line, "%d %d", width, height);
}

static void parseDimensions(const char *line0, int *width, int *height,
                            int *channels) {
  const char *line = skipSpaces(line0);
  sscanf(line, "%d %d %d", width, height, channels);
}

static void parseDepth(const char *line0, int *depth) {
  const char *line = skipSpaces(line0);
  sscanf(line, "%d", depth);
}

static char *nextLine(htkFile_t file) {
  char *line = nullptr;
  while ((line = htkFile_readLine(file)) != nullptr) {
    if (!isComment(line)) {
      break;
    }
  }
  return line;
}

htkImage_t htkPPM_import(const char *filename) {
  htkImage_t img;
  htkFile_t file;
  char *header;
  char *line;
  int ii, jj, kk, channels;
  int width, height, depth;
  unsigned char *charData, *charIter;
  float *imgData, *floatIter;
  float scale;

  img = nullptr;

  file = htkFile_open(filename, "rb");
  if (file == nullptr) {
    printf("Could not open %s\n", filename);
    goto cleanup;
  }

  header = htkFile_readLine(file);
  if (header == nullptr) {
    printf("Could not read from %s\n", filename);
    goto cleanup;
  } else if (strcmp(header, "P6") != 0 && strcmp(header, "P6\n") != 0 &&
             strcmp(header, "P5") != 0 && strcmp(header, "P5\n") != 0 &&
             strcmp(header, "S6") != 0 && strcmp(header, "S6\n") != 0) {
    printf("Could not find magic number for %s\n", filename);
    goto cleanup;
  }

  // P5 are monochrome while P6/S6 are rgb
  // S6 needs to parse number of channels out of file
  if (strcmp(header, "P5") == 0 || strcmp(header, "P5\n") == 0) {
    channels = 1;
    line     = nextLine(file);
    parseDimensions(line, &width, &height);
  } else if (strcmp(header, "P6") == 0 || strcmp(header, "P6\n") == 0) {
    channels = 3;
    line     = nextLine(file);
    parseDimensions(line, &width, &height);
  } else {
    line = nextLine(file);
    parseDimensions(line, &width, &height, &channels);
  }

  // the line now contains the depth information
  line = nextLine(file);
  parseDepth(line, &depth);

  // the rest of the lines contain the data in binary format
  charData = (unsigned char *)htkFile_read(
      file, width * channels * sizeof(unsigned char), height);

  img = htkImage_new(width, height, channels);

  imgData = htkImage_getData(img);

  charIter  = charData;
  floatIter = imgData;

  scale = 1.0f / ((float)depth);

  for (ii = 0; ii < height; ii++) {
    for (jj = 0; jj < width; jj++) {
      for (kk = 0; kk < channels; kk++) {
        *floatIter = ((float)*charIter) * scale;
        floatIter++;
        charIter++;
      }
    }
  }

#ifdef LAZY_FILE_LOAD
  htkDelete(charData);
#endif

cleanup:
  htkFile_close(file);
  return img;
}

void htkPPM_export(const char *filename, htkImage_t img) {
  int ii;
  int jj;
  int kk;
  int depth;
  int width;
  int height;
  int channels;
  htkFile_t file;
  float *floatIter;
  unsigned char *charData;
  unsigned char *charIter;

  file = htkFile_open(filename, "wb+");

  width    = htkImage_getWidth(img);
  height   = htkImage_getHeight(img);
  channels = htkImage_getChannels(img);
  depth    = 255;

  if (channels == 1) {
    htkFile_writeLine(file, "P5");
  } else {
    htkFile_writeLine(file, "P6");
  }
  htkFile_writeLine(file, "#Created via htkPPM Export");
  htkFile_writeLine(file, htkString(width, " ", height));
  htkFile_writeLine(file, htkString(depth));

  charData = htkNewArray(unsigned char, width * height * channels);

  charIter  = charData;
  floatIter = htkImage_getData(img);

  for (ii = 0; ii < height; ii++) {
    for (jj = 0; jj < width; jj++) {
      for (kk = 0; kk < channels; kk++) {
        *charIter = (unsigned char)(_clamp(*floatIter, 0, 1) * depth + 0.5f);
        floatIter++;
        charIter++;
      }
    }
  }

  htkFile_write(file, charData, width * channels * sizeof(unsigned char),
               height);

  htkDelete(charData);
  htkFile_delete(file);

  return;
}
