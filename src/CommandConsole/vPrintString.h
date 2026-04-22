/*
 * vPrintString.h
 *
 * Created: 1-3-2016 20:39:37
 *  Author: Gerard Harkema
 */ 


#ifndef V_PRINT_STRING_H_
#define V_PRINT_STRING_H_

#if defined(__GNUC__)
#define VPRINTSTRING_PRINTF_FORMAT __attribute__((format(__printf__, 1, 2)))
#else
#define VPRINTSTRING_PRINTF_FORMAT
#endif

void vPrintString(const char *format, ...) VPRINTSTRING_PRINTF_FORMAT;


#endif /* V_PRINT_STRING_H_ */
