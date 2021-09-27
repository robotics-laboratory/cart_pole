# Manual fixes to libs

### MPU6050

In lib `MPU6050`:

In file `MPU6050.h` at line `45` change

```cpp
#include <avr/pgmspace.h>
```

to 

```cpp
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif
```

In file `I2Cdev.cpp` at line `300` change

```cpp
for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
```

to

```cpp
for (uint8_t k = 0; k < length; k += min(static_cast<int>(length), BUFFER_LENGTH)) {
```