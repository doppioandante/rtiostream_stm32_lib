/*
 * Copyright 2007-2012 The MathWorks, Inc.
 *
 * File: rtiostream.h
 *
 * Abstract:
 *  Function prototypes and defines for rtIOStream API.
 */

#ifndef RTIOSTREAM_H
#define RTIOSTREAM_H

#include <stddef.h>

#define RTIOSTREAM_ERROR (-1)
#define RTIOSTREAM_NO_ERROR (0)

/* Note: if the functions declared in this file should be compiled into a shared
 * library (e.g. a .dll file on Windows), you must ensure that the functions are
 * externally visible. The procedure to achieve this depends on the compiler and
 * linker you are using. For example, on Windows, you may need to provide an
 * exports definition .def file that lists all of the functions to be
 * exported; see ./rtiostream/rtiostream_pc.def for a suitable .def file.
 */

#ifndef RTIOSTREAMAPI
#define RTIOSTREAMAPI
#endif

#include "rtiostream_hal.h"

#ifndef HAL_UART_HANDLE_TYPEDEF
#warning "Please define HAL_UART_HANDLE_TYPEDEF macro before including rtiostream.h"
typedef struct DummyUartTypedef;
#define HAL_UART_HANDLE_TYPEDEF DummyUartTypedef
#endif

#ifndef HAL_STATUS_TYPEDEF
#warning "Please define HAL_STATUS_TYPEDEF macro before including rtiostream.h"
#define HAL_STATUS_TYPEDEF HAL_StatusTypeDef
#endif

#ifndef HAL_UART_TIMEOUT
#error "Define an apporiate HAL_UART_TIMEOUT"
#endif

RTIOSTREAMAPI int rtIOStreamOpen(
    int    argc,
    void * argv[]
);

RTIOSTREAMAPI int rtIOStreamSend(
    int          streamID,
    const void * src,
    size_t       size,
    size_t     * sizeSent
    );

RTIOSTREAMAPI int rtIOStreamRecv(
    int      streamID,
    void   * dst,
    size_t   size,
    size_t * sizeRecvd
    );

RTIOSTREAMAPI int rtIOStreamClose(
    int streamID
    );


#endif /* #ifndef RTIOSTREAM_H */
