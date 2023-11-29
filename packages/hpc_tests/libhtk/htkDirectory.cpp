#include "htk.h"

#ifndef PATH_MAX
#ifdef FILENAME_MAX
#define PATH_MAX FILENAME_MAX
#else /* FILENAME_MAX */
#define PATH_MAX 4096
#endif /* FILENAME_MAX */
#endif /* PATH_MAX */

#ifdef HTK_USE_UNIX
const char htkDirectorySeperator = '/';
static char *getcwd_(char *buf, int maxLen) {
  return getcwd(buf, maxLen);
}
static void mkdir_(const char *dir) {
  mkdir(dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}
#else  /* HTK_USE_LINUX */
const char htkDirectorySeperator = '\\';
static char *getcwd_(char *buf, int maxLen) {
  return _getcwd(buf, maxLen);
}
static void mkdir_(const char *dir) {
  _mkdir(dir);
}
#endif /* HTK_USE_LINUX */

EXTERN_C const char *htkDirectory_create(const char *dir) {
  char tmp[PATH_MAX];
  char *p = nullptr;
  size_t len;

#ifdef HTK_USE_WINDOWS
  _snprintf(tmp, sizeof(tmp), "%s", dir);
#else /* HTK_USE_WINDOWS */
  snprintf(tmp, sizeof(tmp), "%s", dir);
#endif /* HTK_USE_WINDOWS */
  len = strlen(tmp);
  if (tmp[len - 1] == htkDirectorySeperator) {
    tmp[len - 1] = 0;
  }
  for (p = tmp + 1; *p; p++) {
    if (*p == htkDirectorySeperator) {
      *p = 0;
      mkdir_(tmp);
      *p = htkDirectorySeperator;
    }
  }
  mkdir_(tmp);
  return dir;
}

EXTERN_C char *htkDirectory_name(const char *pth0) {
  char *pth = htkString_duplicate(pth0);
  char *p   = strrchr(pth, htkDirectorySeperator);
  if (p) {
    p[0] = 0;
  }
  return pth;
}

EXTERN_C char *htkDirectory_current() {
  char *tmp = htkNewArray(char, PATH_MAX + 1);
  if (getcwd_(tmp, PATH_MAX)) {
    return tmp;
  }

  htkDelete(tmp);

  int error = errno;
  switch (error) {
    case EACCES:
      std::cerr
          << "Cannot get current directory :: access denied. exiting..."
          << std::endl;
      exit(-1);
    case ENOMEM:
      std::cerr << "Cannot get current directory :: insufficient storage. "
                   "exiting..."
                << std::endl;
      exit(-1);
    default:
      std::cerr << "Cannot get current directory :: unrecognised error "
                << error << std::endl;
      exit(-1);
  }
}
