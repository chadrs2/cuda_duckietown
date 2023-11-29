

#ifndef __htkPPM_H__
#define __htkPPM_H__

START_EXTERN_C
htkImage_t htkPPM_import(const char *filename);
void htkPPM_export(const char *filename, htkImage_t img);
END_EXTERN_C

#endif /* __htkPPM_H__ */
