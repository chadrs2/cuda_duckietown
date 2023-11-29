#ifndef __HTK_DIRECTORY__
#define __HTK_DIRECTORY__

extern const char htkDirectorySeperator;
EXTERN_C char *htkDirectory_name(const char *pth);
EXTERN_C const char *htkDirectory_create(const char *dir);
EXTERN_C char *htkDirectory_current();

#endif /* __HTK_DIRECTORY__ */
