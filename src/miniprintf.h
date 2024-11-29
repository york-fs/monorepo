/* Minimal printf() facility for MCUs
 * Warren W. Gay VE3WWG, Sun Feb 12 2017
 * 
 * This work is placed into the public domain. No warranty, or guarantee
 * is expressed or implied. When uou use this source code, you do so
 * with full responsibility.
 */
#ifndef MINIPRINTF_H
#define MINIPRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

int mini_vprintf_cooked(void (*putc)(char),const char *format,va_list args);
int mini_vprintf_uncooked(void (*putc)(char),const char *format,va_list args);

int mini_snprintf(char *buf,unsigned maxbuf,const char *format,...)
	__attribute((format(printf,3,4)));

#ifdef __cplusplus
}
#endif

#endif // MINIPRINTF_H
