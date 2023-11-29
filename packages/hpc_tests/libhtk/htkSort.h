
#ifndef __HTK_SORT_H__
#define __HTK_SORT_H__


template <typename T>
static void htkSort(T *data, int start, int end) {
  if ((end - start + 1) > 1) {
    int left = start, right = end;
    int pivot = data[right];
    while (left <= right) {
      while (data[left] > pivot) {
        left = left + 1;
      }
      while (data[right] < pivot) {
        right = right - 1;
      }
      if (left <= right) {
        int tmp     = data[left];
        data[left]  = data[right];
        data[right] = tmp;
        left        = left + 1;
        right       = right - 1;
      }
    }
    htkSort(data, start, right);
    htkSort(data, left, end);
  }
}

template <typename T>
static void htkSort(T *data, int len)  {
    return htkSort<T>(data, 0, len);
}

template <typename T, typename K>
static void htkSortByKey(T *data, K *key, int start, int end) {
  if ((end - start + 1) > 1) {
    int left = start, right = end;
    int pivot = key[right];
    while (left <= right) {
      while (key[left] > pivot) {
        left = left + 1;
      }
      while (key[right] < pivot) {
        right = right - 1;
      }
      if (left <= right) {
        int tmp     = key[left];
        key[left]   = key[right];
        key[right]  = tmp;
        tmp         = data[left];
        data[left]  = data[right];
        data[right] = tmp;
        left        = left + 1;
        right       = right - 1;
      }
    }
    htkSortByKey(data, key, start, right);
    htkSortByKey(data, key, left, end);
  }
}

template <typename T, typename K>
static void htkSortByKey(T *data, K *key, int len) {
    return htkSortByKey<T, K>(data, key, len);
}


#endif // __HTK_SORT_H__
