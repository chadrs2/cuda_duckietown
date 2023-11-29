#ifndef __HTK_PATH_H__
#define __HTK_PATH_H__

char *htkPath_join(const char *p1, const char *p2);
char *htkPath_join(const char *p1, const char *p2, const char *p3);
char *htkPath_join(const char *p1, const char *p2, const char *p3,
                  const char *p4);

char *htkPath_join(const std::string &p1, const std::string &p2);
char *htkPath_join(const std::string &p1, const std::string &p2,
                  const std::string &p3);
char *htkPath_join(const std::string &p1, const std::string &p2,
                  const std::string &p3, const std::string &p4);

template <typename T1, typename T2>
static char *htkPath_join(const T1 &p1, const T2 &p2) {
  return htkPath_join(htkString(p1), htkString(p2));
}
template <typename T1, typename T2, typename T3>
static char *htkPath_join(const T1 &p1, const T2 &p2, const T3 &p3) {
  return htkPath_join(htkString(p1), htkString(p2), htkString(p3));
}
template <typename T1, typename T2, typename T3, typename T4>
static char *htkPath_join(const T1 &p1, const T2 &p2, const T3 &p3,
                         const T4 &p4) {
  return htkPath_join(htkString(p1), htkString(p2), htkString(p3),
                     htkString(p4));
}

#endif /* __HTK_PATH_H__ */
