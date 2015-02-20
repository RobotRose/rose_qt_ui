#ifndef TRACE_H
#define TRACE_H

#include <stdio.h>

#define DS_LOG_GEN_TRACE(fmt, ...)/* \
    fprintf(stdout, "%s:%d:\n" fmt, __FILE__, __LINE__, ##__VA_ARGS__)  */

#endif // TRACE_H
