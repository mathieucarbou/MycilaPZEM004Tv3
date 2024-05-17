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
- Focus on speed and reactivity with a callback mechanism
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
  pzem.begin(Serial1, 14, 27);
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
  pzem.begin(Serial1, 14, 27, MYCILA_PZEM_DEFAULT_ADDRESS, true);
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
pzem.begin(Serial1, 14, 27, address);
```

### Json support

Json support is optional.
If you need it, please add this compilation flag to activate it: `-D MYCILA_PZEM_JSON_SUPPORT` and do not forget to include the ArduinoJson library:

```c++
#include <ArduinoJson.h>
```

### Multiple PZEM-004T v3 devices on the same RX/TX port

1. Connect the first PZEM-004T v3 to the RX/TX port of the ESP32
2. Restart and set the address of the first PZEM-004T v3 (you can use the `SetAddress.ino` file)
3. Disconnect the first PZEM-004T v3 and connect the second one instead
4. Restart and set the address of the second PZEM-004T v3
5. Connect both PZEM-004T v3 to the RX/TX port of the ESP32
6. Restart and use both with their own address

### Callbacks

- `PZEMCallback`: called when the PZEM has read the data and when a change for any of the metric is detected.
  This is useful to be notified exactly when required.
  You must check the event type

**Callback Example**

Reading a load for 2 second after it is turned on:

```c++
  pzem.setCallback([](const Mycila::PZEMEventType eventType) {
    int64_t now = esp_timer_get_time();
    switch (eventType) {
      case Mycila::PZEMEventType::EVT_READ:
        Serial.printf(" - %" PRId64 " EVT_READ\n", now);
        break;
      case Mycila::PZEMEventType::EVT_CHANGE:
        Serial.printf(" - %" PRIu32 " EVT_CHANGE: %f V, %f A, %f W\n", millis(), pzem.getVoltage(), pzem.getCurrent(), pzem.getPower());
        break;
      default:
        Serial.printf(" - %" PRId64 " ERR\n", now);
        break;
    }
  });
```

```
 - 227 EVT_CHANGE: 236.199997 V, 0.000000 A, 0.000000 W
 - 287749 EVT_READ
 - 347736 EVT_READ
 - 407822 EVT_READ
 - 468011 EVT_READ
 - 527993 EVT_READ
 - 588081 EVT_READ
 - 648269 EVT_READ
 - 708261 EVT_READ
 - 768343 EVT_READ
 - 828434 EVT_READ
 - 888633 EVT_READ
 - 948717 EVT_READ
 - 1008799 EVT_READ
 - 1068889 EVT_READ
 - 1128980 EVT_READ
 - 1189069 EVT_READ
 - 1249167 EVT_READ
 - 1249 EVT_CHANGE: 233.399994 V, 2.262000 A, 497.899994 W
 - 1309258 EVT_READ
 - 1369438 EVT_READ
 - 1434762 EVT_READ
 - 1494647 EVT_READ
 - 1554730 EVT_READ
 - 1614816 EVT_READ
 - 1674910 EVT_READ
 - 1674 EVT_CHANGE: 233.399994 V, 2.262000 A, 497.899994 W
 - 1734995 EVT_READ
 - 1795086 EVT_READ
 - 1855168 EVT_READ
 - 1915365 EVT_READ
 - 1975447 EVT_READ
 - 2035538 EVT_READ
 - 2095633 EVT_READ
 - 2155718 EVT_READ
 - 2215800 EVT_READ
 - 2275891 EVT_READ
 - 2335982 EVT_READ
 - 2396071 EVT_READ
 - 2461382 EVT_READ
 - 2521268 EVT_READ
 - 2581354 EVT_READ
 - 2641551 EVT_READ
 - 2701644 EVT_READ
 - 2701 EVT_CHANGE: 233.899994 V, 2.261000 A, 539.500000 W
 - 2761619 EVT_READ
 - 2821705 EVT_READ
 - 2881902 EVT_READ
 - 2941987 EVT_READ
 - 3002075 EVT_READ
 - 3062161 EVT_READ
 - 3122255 EVT_READ
 - 3182440 EVT_READ
 - 3242428 EVT_READ
 - 3302514 EVT_READ
 - 3362608 EVT_READ
 - 3422690 EVT_READ
 - 3488113 EVT_READ
 - 3547891 EVT_READ
 - 3608088 EVT_READ
 - 3668170 EVT_READ
 - 3728260 EVT_READ
 - 3788346 EVT_READ
 - 3848441 EVT_READ
 - 3908634 EVT_READ
 - 3908 EVT_CHANGE: 236.600006 V, 0.000000 A, 0.000000 W
```

## Performance tests

Here are below some test results for the PZEM for 50 consecutive reads on a nominal load of about 650W, controlled with a random SSR relay (0-100%).

**PerfTest1**

```
pzem.read():
 - Errors: 0
 - Average read time: 120433 us
 - Min read time: 107118 us
 - Max read time: 126020 us
```

**PerfTest2**

```
pzem.read():
 - ROUND: 1
   * Load Detection time: 1207002 us (10 reads)
   * Ramp up time: 2534171 us (21 reads)
   * Ramp down time: 2537724 us (21 reads)
 - ROUND: 2
   * Load Detection time: 1328073 us (11 reads)
   * Ramp up time: 2655242 us (22 reads)
   * Ramp down time: 2539094 us (21 reads)
 - ROUND: 3
   * Load Detection time: 1206870 us (10 reads)
   * Ramp up time: 2534040 us (21 reads)
   * Ramp down time: 2537968 us (21 reads)
 - ROUND: 4
   * Load Detection time: 1328062 us (11 reads)
   * Ramp up time: 2655334 us (22 reads)
   * Ramp down time: 2538888 us (21 reads)
```

The "Load Detection time" is the time it takes for the PZEM to see a load `> 0` just after the relay was closed on a constant resistive load.

The "Ramp up time" is the time it takes for the PZEM to stabilize (with a maximum delta of 1% from last measurement) and see an expected load value.

The "Ramp down time" is the time it takes for the PZEM to return to 0W after the relay was opened.

You can run the `PerfTests` from the examples.
The numbers might change a little but the order of magnitude should stay the same.

**What these results mean ?**

- It takes **more than 1.2 second** for the PZEM to report a change of a load appearing on the wire.
  Reading the PZEM more frequently (in a async loop) and faster will allow a change to be seen as fast as possible, but it will still take at least 1 second for the change to be seen.
  **The best reactivity is achieved when reading the PZEM in a loop, and using callback mechanism.**

- It takes **more than 2.5 seconds** for the PZEM to report a load value which is stable and according to what we expect.
  This duration contains the duration for the load to reach its nominal power, plus the duration it takes for the PZEM to stabilize its measurements.

Te PZEM might use a sort of moving average over a window of about 2 seconds with updates every 1 second.

## Reference material

- [PZEM-004T-V3.0-Datasheet-User-Manual.pdf](https://oss.carbou.me/MycilaPZEM004Tv3/PZEM-004T-V3.0-Datasheet-User-Manual.pdf)
- [PZEM-004T V3](https://innovatorsguru.com/pzem-004t-v3/)
- [mandulaj/PZEM-004T-v30](https://github.com/mandulaj/PZEM-004T-v30)
