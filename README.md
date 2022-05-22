# serial rtiostream implementation on STM32 HAL

## How to use
Copy the files inside your STM32 project. Change `rtiostream_hal.h` so that it includes your STM HAL header.
You may have to tweak the timeout used by the UART transmit/receive functions.

## Testing
It's suggested to first test the library in a standalone application.
Include `rtiostreamtest.h` and execute `rtiostreamtest()` in your main code, then use [rtiostreamtest](https://it.mathworks.com/help/ecoder/ref/rtiostreamtest.html)
to verify that it works.
In a standard STM32CubeIde project, USER CODE BEGIN 2 would be a good section:
```c
/* USER CODE BEGIN 2 */
rtiostreamtest(0, NULL);
/* USER CODE END 2 */
```

## LICENSE
`rtiostream.h`, `rtiostream_hal.h`, `rtiostreamtest.c` and `rtiostreamtest.h` are copied and licensed by MathWorks.
`rtiostream_stm32_impl.c` is under MIT license.
