#ifndef RTIOSTREAM_STM32_H_INCLUDED
#define RTIOSTREAM_STM32_H_INCLUDED

#include <assert.h>
#include <stdint.h>
#include "rtiostream.h"

#define RTIOSTREAM_STM32_CONST_ID 0xfd

extern HAL_UART_HANDLE_TYPEDEF* rtiostream_huart_handle;

HAL_STATUS_TYPEDEF HAL_UART_Transmit(HAL_UART_HANDLE_TYPEDEF *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_STATUS_TYPEDEF HAL_UART_Receive(HAL_UART_HANDLE_TYPEDEF *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

int rtIOStreamOpen(int argc, void *argv[]) {
	return RTIOSTREAM_STM32_CONST_ID;
}

int rtIOStreamSend(int streamID, const void *src, size_t size, size_t* sizeSent) {
	assert(streamID == RTIOSTREAM_STM32_CONST_ID);

	if (size == 0) {
		*sizeSent = 0;
		return RTIOSTREAM_NO_ERROR;
	}

	HAL_STATUS_TYPEDEF res = HAL_UART_Transmit(rtiostream_huart_handle, (const void*)src, (size_t) size, HAL_UART_TIMEOUT);
	if (res == HAL_OK) {
		*sizeSent = size;
		return RTIOSTREAM_NO_ERROR;
	} else {
		*sizeSent = 0;
		return RTIOSTREAM_ERROR;
	}
}

int rtIOStreamRecv(int streamID, void* dst, size_t size, size_t * sizeRecvd) {
	assert(streamID == RTIOSTREAM_STM32_CONST_ID);
	assert(size <= UINT16_MAX);

	if (size == 0) {
		*sizeRecvd = 0;
		return RTIOSTREAM_NO_ERROR;
	}

	HAL_STATUS_TYPEDEF res = HAL_UART_Receive(rtiostream_huart_handle, (uint8_t*) dst, (uint16_t) size, HAL_UART_TIMEOT);
	if (res == HAL_OK) {
		*sizeRecvd = size;
		return RTIOSTREAM_NO_ERROR;
	} else if (res == HAL_TIMEOUT) {
		*sizeRecvd = 0;
		return RTIOSTREAM_NO_ERROR;
	} else {
		*sizeRecvd = 0;
		return RTIOSTREAM_ERROR;
	}
}

int rtIOStreamClose(int streamID) {
	assert(streamID == RTIOSTREAM_STM32_CONST_ID);

	return RTIOSTREAM_NO_ERROR;
}

#endif
