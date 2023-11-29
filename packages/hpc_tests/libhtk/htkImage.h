

#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "htkTypes.h"

struct st_htkImage_t {
  int width;
  int height;
  int channels;
  int pitch;
  float *data;
};

#define htkImage_channels 3

#define htkImage_getWidth(img) ((img)->width)
#define htkImage_getHeight(img) ((img)->height)
#define htkImage_getChannels(img) ((img)->channels)
#define htkImage_getPitch(img) ((img)->pitch)
#define htkImage_getData(img) ((img)->data)

#define htkImage_setWidth(img, val) (htkImage_getWidth(img) = val)
#define htkImage_setHeight(img, val) (htkImage_getHeight(img) = val)
#define htkImage_setChannels(img, val) (htkImage_getChannels(img) = val)
#define htkImage_setPitch(img, val) (htkImage_getPitch(img) = val)
#define htkImage_setData(img, val) (htkImage_getData(img) = val)

typedef void (*htkImage_onSameFunction_t)(string str);

htkImage_t htkImage_new(int width, int height, int channels, float *data);
htkImage_t htkImage_new(int width, int height, int channels);
htkImage_t htkImage_new(int width, int height);
void htkImage_delete(htkImage_t img);
htkBool htkImage_sameQ(htkImage_t a, htkImage_t b,
                     htkImage_onSameFunction_t onUnSame);
htkBool htkImage_sameQ(htkImage_t a, htkImage_t b);

#endif /* __IMAGE_H__ */
