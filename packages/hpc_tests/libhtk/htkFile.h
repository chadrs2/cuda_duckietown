

#ifndef __HTK_FILE_H__
#define __HTK_FILE_H__

struct st_htkFile_t {
  int index;
  char *file;
  char *mode;
  char *data;
  FILE *handle;
  size_t len;
  size_t offset;
};

#define htkFile_getIndex(file) ((file)->index)
#define htkFile_getFileName(file) ((file)->file)
#define htkFile_getMode(file) ((file)->mode)
#define htkFile_getData(file) ((file)->data)
#define htkFile_getLength(file) ((file)->len)
#define htkFile_getDataOffset(file) ((file)->offset)
#define htkFile_getFileHandle(file) ((file)->handle)

#define htkFile_setIndex(file, val) (htkFile_getIndex(file) = val)
#define htkFile_setFileName(file, val) (htkFile_getFileName(file) = val)
#define htkFile_setMode(file, val) (htkFile_getMode(file) = val)
#define htkFile_setData(file, val) (htkFile_getData(file) = val)
#define htkFile_setLength(file, val) (htkFile_getLength(file) = val)
#define htkFile_setDataOffset(file, val) (htkFile_getDataOffset(file) = val)
#define htkFile_setFileHandle(file, val) (htkFile_getFileHandle(file) = val)

htkFile_t htkFile_new(void);
void htkFile_delete(htkFile_t file);
void htkFile_close(htkFile_t file);
void htkFile_init(void);
void htkFile_atExit(void);
int htkFile_count(void);
htkFile_t htkFile_open(const char *fileName, const char *mode);
htkFile_t htkFile_open(const char *fileName);
char *htkFile_read(htkFile_t file, size_t size, size_t count);
char *htkFile_read(htkFile_t file, size_t len);
void htkFile_rewind(htkFile_t file);
size_t htkFile_size(htkFile_t file);
char *htkFile_read(htkFile_t file);
char *htkFile_readLine(htkFile_t file);
void htkFile_write(htkFile_t file, const void *buffer, size_t size,
                  size_t count);
void htkFile_write(htkFile_t file, const void *buffer, size_t len);
void htkFile_write(htkFile_t file, const char *buffer);
void htkFile_writeLine(htkFile_t file, const char *buffer0);
void htkFile_write(htkFile_t file, string buffer);
void htkFile_writeLine(htkFile_t file, string buffer0);
htkBool htkFile_existsQ(const char *path);
char *htkFile_extension(const char *file);

#endif /* __HTK_FILE_H__ */
