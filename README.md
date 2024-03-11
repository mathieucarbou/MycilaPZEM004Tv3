# MycilaPZEM004Tv3

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Continuous Integration](https://github.com/mathieucarbou/MycilaPZEM004Tv3/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaPZEM004Tv3/actions/workflows/ci.yml)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/mathieucarbou/library/MycilaPZEM004Tv3.svg)](https://registry.platformio.org/libraries/mathieucarbou/MycilaPZEM004Tv3)

Arduino / ESP32 library for the PZEM-004T v3 Power and Energy monitor

Please read this good article first:

- [PZEM-004T V3](https://innovatorsguru.com/pzem-004t-v3/)

And also have a look at this inspiring and well documented GitHub project:

- [mandulaj/PZEM-004T-v30](https://github.com/mandulaj/PZEM-004T-v30)

This project is an adaptation of the project above, focusing only on ESP32 and add support for async reading and async energy reset.

- **Support multiple PZEM-004T v3 devices on the same RX/TX port**
- Support reading and setting addresses of PZEM-004T v3 devices
- Blocking and non-blocking mode
- Core, stack size, priority and read timeout can be configured
- Automatically detect / search for PZEM-004T v3 devices
- Energy reset live at runtime, in both async and normal mode
- Available metrics:

```c++
    volatile float current = 0; // A
    volatile float energy = 0; // kWh
    volatile float power = 0; // W
    volatile float powerFactor = 0;
    volatile float voltage = 0; // V
    volatile uint8_t frequency = 0; // Hz
```

## Usage

Have a look at all the examples in the [examples](examples) folder.

There is a getter for each metric.

### Blocking mode

```c++
Mycila::PZEM pzem;

void setup() {
  pzem.begin(&Serial1, 27, 14);
}

void loop() {
  if (pzem.read()) {
    // access values
  }
  delay(1000);
}
```

### Non-Blocking mode (async)

```c++
Mycila::PZEM pzem;

void setup() {
  pzem.begin(&Serial1, 27, 14, MYCILA_PZEM_DEFAULT_ADDRESS, true);
}
```

Calling read() is not necessary.

### Energy reset

```c++
pzem.resetEnergy();
```

### Read and set address

```c++
pzem.readAddress();
pzem.readAddress(true); // will read teh address and use it as the new address

pzem.setAddress(0x42);
```

### Start a PZEM with a specific address

```c++
pzem.begin(&Serial1, 27, 14, address);
```

### Json support

Json support is optional. 
If you need it, please add this compilation flag to activate it: `-D MYCILA_PZEM_JSON_SUPPORT` and do not forget to include the ArduinoJson library:

```c++
#include <ArduinoJson.h>
```

## Multiple PZEM-004T v3 devices on the same RX/TX port

1. Connect the first PZEM-004T v3 to the RX/TX port of the ESP32
2. Restart and set the address of the first PZEM-004T v3 (you can use the `SetAddress.ino` file)
3. Disconnect the first PZEM-004T v3 and connect the second one instead
4. Restart and set the address of the second PZEM-004T v3
5. Connect both PZEM-004T v3 to the RX/TX port of the ESP32
6. Restart and use both with their own address

## Reference material

- [PZEM-004T-V3.0-Datasheet-User-Manual.pdf](https://oss.carbou.me/MycilaPZEM004Tv3/PZEM-004T-V3.0-Datasheet-User-Manual.pdf)
- [PZEM-004T V3](https://innovatorsguru.com/pzem-004t-v3/)
- [mandulaj/PZEM-004T-v30](https://github.com/mandulaj/PZEM-004T-v30)
