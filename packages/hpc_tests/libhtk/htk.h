#ifndef __HTK_H__
#define __HTK_H__

/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _MSC_VER

// set minimal warning level
#pragma warning(push, 0)
// some warnings still occur at this level
// if necessary, disable specific warnings not covered by previous pragma
#pragma warning( \
    disable : 4244 4056 4305 4800 4267 4996 4756 4661 4385 4101 4800)

#define NOMINMAX // do not define min/max in the standard headers

#define __func__ __FUNCTION__
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif /* _CRT_SECURE_NO_WARNINGS */
#define _CRT_SECURE_NO_DEPRECATE 1
#define _CRT_NONSTDC_NO_DEPRECATE 1
#include <direct.h>
#include <io.h>
#include <windows.h>
#define HTK_USE_WINDOWS
#else /* _MSC_VER */
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#define HTK_USE_UNIX
#ifdef __APPLE__
#include <mach/mach_time.h>
#define HTK_USE_DARWIN
#else /* __APPLE__ */
#define HTK_USE_LINUX
#endif /* __APPLE__ */
#endif /* _MSC_VER */

#define htkStmt(stmt) stmt

/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/

#define htkLine __LINE__
#define htkFile __FILE__
#define htkFunction __func__

#define htkExit() \
  htkAssert(0);   \
  exit(1)

/* Printing and logging options */
// #define htkLogger_print 1
// #define htkLogger_log 1
// #define htkTimer_print 1
// #define htkTimer_log 1
// #define htkSolution_print 1
// #define htkSolution_log 1

/* Output format when printing, otherwise simple text */
// #define JSON_OUTPUT 1
// TODO: XML_OUTPUT

/* Print at exit with JSON output */
#ifdef HTK_USE_COURSERA
#define htkExit_print 1
#define JSON_OUTPUT 1
#else
/* Print on call with text output, no log entries saved */
#define htkLogger_print 1
#define htkTimer_print 1
#define htkSolution_print 1
#endif /* HTK_USE_COURSERA */

/* If printing at exit, make sure log entries are saved */
#ifdef htkExit_print
#define htkLogger_log 1
#endif /* htkExit_print */

/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/

#ifdef __cplusplus
#define EXTERN_C extern "C"
#define START_EXTERN_C EXTERN_C {
#define END_EXTERN_C }
#else
#define EXTERN_C
#define START_EXTERN_C
#define END_EXTERN_C
#endif /* __cplusplus */

/***********************************************************/
/***********************************************************/
/***********************************************************/

#ifndef HTK_USE_JSON11
#define HTK_USE_JSON11 1
#endif /* HTK_USE_JSON11 */

#if 1 || HTK_USE_JSON11
#include "vendor/json11.hpp"
#endif /* HTK_USE_JSON11 */

/***********************************************************/
/***********************************************************/
/***********************************************************/

#define LAZY_FILE_LOAD
extern char *solutionJSON;

/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/

#ifdef HTK_USE_OPENCL
#ifdef HTK_USE_DARWIN
#include <OpenCL/opencl.h>
#else /* HTK_USE_DARWIN */
#include <CL/cl.h>
#endif /* HTK_USE_DARWIN */
#endif /* HTK_USE_OPENCL */

#include "htkTypes.h"

#include "htkAssert.h"
#include "htkMalloc.h"
#include "htkString.h"
#include "htkUtils.h"

#include "htkArg.h"
#include "htkCUDA.h"
#include "htkCast.h"
#include "htkComparator.h"
#include "htkDirectory.h"
#include "htkExit.h"
#include "htkExport.h"
#include "htkFile.h"
#include "htkImage.h"
#include "htkImport.h"
#include "htkInit.h"
#include "htkLogger.h"
#include "htkMD5.h"
#include "htkMPI.h"
#include "htkSolution.h"
#include "htkSort.h"
#include "htkSparse.h"
#include "htkThrust.h"
#include "htkTimer.h"
#include "htkPath.h"

#include "htkDataset.h"

/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/
/***********************************************************/

#endif /* __HTK_H__ */
