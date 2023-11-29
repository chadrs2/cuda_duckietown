

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

htkImage_t htkImage_new(int width, int height, int channels, float *data) {
  htkImage_t img;

  img = htkNew(struct st_htkImage_t);

  htkImage_setWidth(img, width);
  htkImage_setHeight(img, height);
  htkImage_setChannels(img, channels);
  htkImage_setPitch(img, width * channels);

  htkImage_setData(img, data);
  return img;
}

htkImage_t htkImage_new(int width, int height, int channels) {
  float *data = htkNewArray(float, width *height *channels);
  return htkImage_new(width, height, channels, data);
}

htkImage_t htkImage_new(int width, int height) {
  return htkImage_new(width, height, htkImage_channels);
}

void htkImage_delete(htkImage_t img) {
  if (img != nullptr) {
    if (htkImage_getData(img) != nullptr) {
      htkDelete(htkImage_getData(img));
    }
    htkDelete(img);
  }
}

static inline void htkImage_setPixel(htkImage_t img, int x, int y, int c,
                                    float val) {
  float *data  = htkImage_getData(img);
  int channels = htkImage_getChannels(img);
  int pitch    = htkImage_getPitch(img);

  data[y * pitch + x * channels + c] = val;

  return;
}

static inline float htkImage_getPixel(htkImage_t img, int x, int y, int c) {
  float *data  = htkImage_getData(img);
  int channels = htkImage_getChannels(img);
  int pitch    = htkImage_getPitch(img);

  return data[y * pitch + x * channels + c];
}

htkBool htkImage_sameQ(htkImage_t a, htkImage_t b,
                     htkImage_onSameFunction_t onUnSame) {
  if (a == nullptr || b == nullptr) {
    htkLog(ERROR, "Comparing null images.");
    return htkFalse;
  } else if (a == b) {
    return htkTrue;
  } else if (htkImage_getWidth(a) != htkImage_getWidth(b)) {
    htkLog(ERROR, "Image widths do not match.");
    return htkFalse;
  } else if (htkImage_getHeight(a) != htkImage_getHeight(b)) {
    htkLog(ERROR, "Image heights do not match.");
    return htkFalse;
  } else if (htkImage_getChannels(a) != htkImage_getChannels(b)) {
    htkLog(ERROR, "Image channels do not match.");
    return htkFalse;
  } else {
    float *aData, *bData;
    int width, height, channels;
    int ii, jj, kk;

    aData = htkImage_getData(a);
    bData = htkImage_getData(b);

    htkAssert(aData != nullptr);
    htkAssert(bData != nullptr);

    width    = htkImage_getWidth(a);
    height   = htkImage_getHeight(a);
    channels = htkImage_getChannels(a);

    for (ii = 0; ii < height; ii++) {
      for (jj = 0; jj < width; jj++) {
        for (kk = 0; kk < channels; kk++) {
          float x, y;
          if (channels <= 3) {
            x = _clamp(*aData++, 0, 1);
            y = _clamp(*bData++, 0, 1);
          } else {
            x = *aData++;
            y = *bData++;
          }
          if (htkUnequalQ(x, y)) {
            if (onUnSame != nullptr) {
              string str = htkString(
                  "Image pixels do not match at position ( row = ",
                  htkString(ii, ", col = ", jj, ", channel = ", kk,
                           ") expecting a value of "),
                  htkString(y, " but got a computed value of ", x));
              onUnSame(str);
            }
            return htkFalse;
          }
        }
      }
    }
    return htkTrue;
  }
}

static void htkImage_onUnsameFunction(string str) {
  htkLog(ERROR, str);
}

htkBool htkImage_sameQ(htkImage_t a, htkImage_t b) {
  return htkImage_sameQ(a, b, htkImage_onUnsameFunction);
}
