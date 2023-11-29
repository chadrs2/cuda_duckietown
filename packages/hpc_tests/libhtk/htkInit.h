

#ifndef __HTK_INIT_H__
#define __HTK_INIT_H__

#ifndef _MSC_VER
__attribute__((__constructor__))
#endif /* _MSC_VER */
void htk_init(int *argc, char ***argv);

#endif /* __HTK_INIT_H__ */
