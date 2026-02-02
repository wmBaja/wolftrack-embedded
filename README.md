# Wolftrack Embedded (Baja CAN Nodes)

Firmware for the team’s CAN-connected nodes (sensors, loggers, etc.) running on an AVR128DB32 with an MCP251863 CAN-FD controller. The PlatformIO project lives in `bajacan/` and is kept small so new contributors can get productive quickly.

## Project Layout
- `bajacan/src/main.cpp`: Main loop that initializes CAN, polls sensors, handles sleep/wake commands, and sends sensor frames.
- `bajacan/include/config.h`: Shared contracts (board pins, control message IDs, sensor descriptor shape, default CAN settings).
- `bajacan/config/`: Board-specific configs. `sensors_config.h` picks the board file that sets `kBoardConfig`. `board_example.h` is a template.
- `bajacan/lib/`: Reusable libraries. `can_driver/` wraps MCP251863 sleep/normal modes. Add sensor libraries here too (see “Creating a Sensor Library”).
- `bajacan/platformio.ini`: Build environments. `env:board_example` shows how to select a board config with `-DBOARD_CONFIG_HEADER="board_example.h"`.

## Quick Start
1) Install PlatformIO (VS Code extension or CLI).  
2) From the repo root: `cd bajacan`.  
3) Build the example config: `pio run -e board_example`.  
4) Upload to a connected board: `pio run -t upload -e board_example`.  
5) Use the Serial Monitor or a CAN tool to watch traffic while powering the node.

## How the Firmware Works
- `setup()`: Configures SPI and CAN pins, starts the MCP251863 driver, then calls each sensor’s `begin` hook.
- `loop()`: Continuously services incoming CAN frames, wakes from sleep on a wake command, polls sensors on their intervals, sends samples, and enters sleep when commanded.
- Sleep/wake is CAN-driven: frames matching the IDs/bytes in `BoardConfig::control` set `gSleepRequested`/`gWakeRequested`. While sleeping, sensors are suspended and the CAN controller is in low-power mode.
- Optional hooks (`BoardHooks`) let a board run custom code before setup, right before sleep, and right after wake.

## Defining a Board
Board configs are plain headers in `bajacan/config/` that populate a single `constexpr BoardConfig kBoardConfig`. The active header is chosen by the `BOARD_CONFIG_HEADER` build flag (see `platformio.ini`).

Basic steps to add a board:
1) Copy `bajacan/config/board_example.h` to a new file (e.g., `my_board.h`).  
2) Set pin numbers for `canCsPin`, `canIntPin`, and `canStbyPin` if they differ from the defaults.  
3) Adjust CAN timing if needed (`canOscillatorHz`, `arbitrationBitrate`, `dataBitrateFactor`, `useExtendedIds`).  
4) Fill out `control` with the CAN IDs/payload bytes that should trigger sleep/wake.  
5) Provide any `BoardHooks` you want (or use `nullptr`).  
6) Include the sensor headers you need (each sensor library exports a `SensorDescriptor`) and build the `kBoardConfig.sensors` table from those descriptors.  
7) Add a new PlatformIO environment that sets `-DBOARD_CONFIG_HEADER="my_board.h"` so the build picks it up.

### Adding a PlatformIO environment
Create a new environment in `bajacan/platformio.ini` that extends the base AVR settings and points the build at your board header:
```
[env:my_board]
extends = env:AVR128DB32
build_flags =
  -Iinclude
  -Iconfig
  -DBOARD_CONFIG_HEADER=\"my_board.h\"
; Optional: customize upload/monitor settings for your dev board
; upload_port = /dev/tty.usbmodemXXXX
; monitor_speed = 115200
```
Build or upload with `pio run -e my_board` / `pio run -t upload -e my_board`. In VS Code, pick the `my_board` environment from the PlatformIO toolbar so uploads/monitoring use the right config.

### Example board config (`bajacan/config/my_board.h`)
```cpp
#pragma once
#include <config.h>
#include <brake_pressure_sensor.h>  // from lib/brake_pressure_sensor
#include <imu_sensor.h>             // from lib/imu_sensor

constexpr BoardHooks kMyHooks{
    .preSetup = nullptr,
    .beforeSleep = nullptr,
    .afterWake = nullptr,
};

constexpr SensorDescriptor kMySensors[] = {
    kBrakePressureSensor,
    kImuSensor,
};

constexpr BoardConfig kBoardConfig{
    kDefaultCanCsPin,
    kDefaultCanIntPin,
    kDefaultCanStbyPin,
    kDefaultMcpOscHz,
    kDefaultArbitrationBitrate,
    kDefaultDataBitrateFactor,
    kDefaultUseExtendedIds,
    {
        .sleepCommandId = 0x100,
        .wakeCommandId = 0x101,
        .sleepCommandByte = 0x0,
        .wakeCommandByte = 0x1,
        .commandByteIndex = 0,
    },  // or, use kDefaultControlCommands
    kMyHooks,
    kMySensors,
    sizeof(kMySensors) / sizeof(kMySensors[0]),
};
```

## Creating a Sensor Library (preferred flow)
Keep sensor implementations in `bajacan/lib/<sensor_name>/` so they can be reused across boards. Each sensor library should:
- Export a `constexpr SensorDescriptor` in `include/<sensor_name>.h` if the board config is `constexpr`.
- Implement the driver logic in `src/<sensor_name>.cpp`.
- Let board configs include the header and drop the exported descriptor into their sensor table.
- If the sensor depends on third-party libraries, add a `library.json` with `dependencies` so PlatformIO compiles it with the right include paths.

### Minimal sensor library example
`lib/throttle_sensor/include/throttle_sensor.h`
```cpp
#pragma once
#include <config.h>

bool ThrottleBegin(void *ctx);
bool ThrottleSample(void *ctx, CANFDMessage &outFrame);

constexpr SensorDescriptor kThrottleSensor{
    .name = "Throttle",
    .canId = 0x200,
    .pollIntervalMs = 20,
    .context = nullptr,
    .begin = ThrottleBegin,
    .sample = ThrottleSample,
    .suspend = nullptr,
    .resume = nullptr,
};
```

`lib/throttle_sensor/src/throttle_sensor.cpp`
```cpp
#include <Arduino.h>
#include <throttle_sensor.h>

static uint16_t gLastReading = 0;

bool ThrottleBegin(void *) {
  pinMode(A0, INPUT);
  return true;  // Return false if init fails.
}

bool ThrottleSample(void *, CANFDMessage &outFrame) {
  gLastReading = analogRead(A0);
  outFrame.len = 2;
  outFrame.data[0] = gLastReading >> 8;
  outFrame.data[1] = gLastReading & 0xFF;
  return true;  // Return false to skip sending.
}
```

Once the library exists, the board config simply includes `<throttle_sensor.h>` and adds `kThrottleSensor` to its `kBoardConfig.sensors` array. If the sensor depends on third-party libraries, add them in `platformio.ini` or the sensor library’s own `library.json`.

### PlatformIO dependency notes
- If a sensor header is only included via a board config header, the LDF may not pick it up; add `#include <sensor_name.h>` to `bajacan/src/main.cpp` or set `lib_ldf_mode = deep+` for that environment.
- If a local library depends on another library (e.g., `ACAN2517FD`), add a `library.json` in that library so PlatformIO pulls in the dependency when compiling it.

## Sleep/Wake Control Frames
- Defaults (`kDefaultControlCommands` in `config.h`): wake byte is `0x1`, sleep byte is `0x0`, both at payload index 0. IDs default to `0x0` and should be overridden per board.  
- `useExtendedIds` determines whether standard or extended IDs are expected for control frames and sensor frames.  
- When a sleep frame arrives, sensors are suspended and the MCP251863 is put into sleep mode. A wake frame resumes everything and resets each sensor’s poll timer.

## Tips for New Contributors
- Start from `board_example.h` and only change one thing at a time.  
- If a sensor isn’t sending, check that `sample` sets `frame.len` and returns `true`.  
- If CAN init fails in `setup()`, the board will sit in an infinite delay—double-check wiring for CS/INT/STBY and oscillator value.  
- Keep IDs unique across sensors to avoid bus conflicts.
