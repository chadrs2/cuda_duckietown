

#ifndef __HTK_CAST_H__
#define __HTK_CAST_H__

template <typename X, typename Y>
static inline void htkCast(X &x, const Y &y, size_t len) {
  size_t ii;

  for (ii = 0; ii < len; ii++) {
    x[ii] = (X)y[ii];
  }

  return;
}

template <typename X, typename Y>
static inline X *htkCast(const Y &y, size_t len) {
  size_t ii;
  X *x = htkNewArray(X, len);

  for (ii = 0; ii < len; ii++) {
    x[ii] = (X)y[ii];
  }

  return x;
}

#endif /* __HTK_CAST_H__ */
