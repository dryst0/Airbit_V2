# Airbit V2 — Drone Flight Controller

A flight controller for the [Airbit V2 drone kit](https://gomakekit.com), built with [MakeCode](https://makecode.microbit.org/) for the BBC micro:bit.

This codebase is written so that a child can read it and understand how a drone works.

---

## How Does a Drone Fly?

A drone has **4 motors**, one on each corner. Each motor spins a propeller that pushes air downward, creating lift (like a helicopter). By spinning some motors faster than others, the drone can tilt and move in any direction.

```
Motor B (front-left)      Motor D (front-right)
         \                    /
          \    micro:bit     /
           +----------------+
          /                  \
         /                    \
Motor A (back-left)       Motor C (back-right)
```

There are three ways a drone can move:

- **ROLL** = tilt left or right (like leaning sideways on a bicycle)
- **PITCH** = tilt forward or backward (like leaning forward to run faster)
- **YAW** = spin left or right (like a spinning top)
- **THROTTLE** = how much power goes to all motors (up = more power, down = less)

The pilot sends commands from a remote controller over **radio**. This code reads those commands, checks the drone's sensors, and adjusts the motor speeds to keep the drone stable in the air.

---

## What Keeps the Drone From Falling Over?

A drone is like balancing a pencil on your finger — it wants to tip over constantly. Without a computer making corrections hundreds of times per second, it would crash instantly. The flight controller uses three key systems:

### 1. Sensors (Knowing Which Way Is Up)

The drone has a tiny chip called an **IMU** (Inertial Measurement Unit) that contains two sensors:

- **Gyroscope** — measures how fast the drone is spinning. Think of spinning a basketball on your finger — the gyroscope measures how quickly it turns. Very accurate for quick movements, but slowly "drifts" over time (like a clock that gains a few seconds each hour).

- **Accelerometer** — measures which direction gravity pulls. When the drone tilts, gravity pulls sideways a little, and the accelerometer detects that tilt. Doesn't drift, but motor vibrations make it jittery during flight.

Neither sensor is perfect alone. The code combines both using a **complementary filter**: 99% gyroscope (for smooth, fast response) + 1% accelerometer (to gently correct the drift). It's like navigating with a compass that wobbles and a step-counter that drifts — together, they keep you on track.

### 2. PID Stabilization (Correcting the Tilt)

Once we know the angle, we need to fix it. **PID** stands for:

- **P (Proportional)** — "How far off am I?" If the drone is tilted 10 degrees, push back hard. If only 1 degree, push back gently. Like tilting a plate to roll a ball back to the center.

- **I (Integral)** — "Has this error been building up?" If the drone keeps drifting slightly left despite the P correction, the integral slowly pushes harder and harder until the drift stops. Like leaning more and more into a wind.

- **D (Derivative)** — "How fast is the error changing?" If the drone is swinging back quickly, slow down the correction so it doesn't overshoot. Like catching a ball — you slow your hand as the ball arrives.

The PID algorithm outputs a correction for each axis (roll, pitch, yaw). These corrections are mixed into the motor speeds:

```
motorA = throttle + rollCorrection + pitchCorrection + yawCorrection  (back-left)
motorB = throttle + rollCorrection - pitchCorrection - yawCorrection  (front-left)
motorC = throttle - rollCorrection + pitchCorrection - yawCorrection  (back-right)
motorD = throttle - rollCorrection - pitchCorrection + yawCorrection  (front-right)
```

### 3. Safety Systems

- **Flip detection** — if the drone tilts past 90 degrees, motors shut off immediately
- **Failsafe** — if the radio signal is lost, the drone slowly descends and then disarms
- **Arm switch** — motors won't spin until the pilot deliberately enables them (like a safety on a toy gun)
- **Low battery protection** — throttle is capped at 75% when the battery is low to prevent a sudden voltage drop

---

## How the Chips Talk to Each Other (I2C)

The micro:bit needs to talk to several tiny chips on the drone board. It does this using **I2C** (pronounced "I-squared-C"), a communication system that works like a shared telephone line with just two wires:

- **SDA** (data wire) — carries the actual messages back and forth
- **SCL** (clock wire) — keeps everyone in sync, like a ticking metronome

Every chip has its own address (like a phone number). When the micro:bit wants to talk to a specific chip, it dials that address first, then sends or asks for data. Only the chip with that address responds — the others stay quiet.

**The chips on our drone:**

| Chip | I2C Address | What It Does |
|------|-------------|--------------|
| Gyroscope (MPU-6050) | 104 | Measures tilt and rotation — like the inner ear that helps you balance |
| Motor Controller (PCA9685) | 98 | Controls the speed of all 4 motors (we send 0-255 and it handles the power) |
| Barometer | 99 | Measures air pressure to estimate height above ground |

---

## Project Files

| File | What It Does |
|------|--------------|
| `main.ts` | The main program — reads radio commands, runs the flight loop, shows the display. **Start reading here.** |
| `custom.ts` | The `airbit` library — talks to the hardware chips (gyroscope, motors, barometer) and runs the PID algorithm |
| `test.ts` | Tests that verify the stabilization and battery math work correctly |
| `pxt.json` | MakeCode project configuration |

---

## Startup Sequence

When the drone powers on, it goes through these steps (shown on the LED screen):

1. **G** — Gyroscope found and configured
2. **M** — Motor controller found and configured
3. **C** — Calibrating gyroscope (drone must be still and level!)
4. **Checkmark** — Ready to fly

If you see "NG" or "No PCA!", a hardware chip wasn't found. Check the wiring.

---

## Display Modes

Press **Button A** or **Button B** to cycle through display modes:

| Mode | What It Shows |
|------|---------------|
| 0 | Joystick position (dot), throttle bar (left), battery bar (right) |
| 1 | Battery level as a bar graph |
| 2 | Battery voltage in millivolts |
| 3 | Raw analog reading from pin P0 |
| 4 | Current throttle value |
| 5 | Motor test (spins each motor one at a time) |
| 6 | Motor speeds as LED brightness in each corner |

Press **A+B** together to reset to mode 0.

---

## Radio Commands

The remote controller sends these values over radio (group 7):

| Name | What It Controls |
|------|-----------------|
| `T` | Throttle (0-100) |
| `P` | Pitch (forward/back tilt) |
| `R` | Roll (left/right tilt) |
| `Y` | Yaw (spin left/right) |
| `A` | Arm (0 = safe, 1 = motors enabled) |

---

## Building from Source

### Prerequisites

- [Node.js](https://nodejs.org/) (v14 or later)

### Build

```bash
npm install
npx mkc build
```

### Open in MakeCode

Import this URL in the MakeCode editor:

```
https://github.com/dryst0/Airbit_V2#cloud-airbitv2
```

Or open [MakeCode](https://makecode.microbit.org/), click **Import** > **Import URL**, and paste the URL above.

### Run Tests

Tests can only run inside the MakeCode simulator or on hardware — there is no headless test runner. Open the project in MakeCode and switch to `test.ts` to run them.

---

## Use as Extension

This repository can be added as an **extension** in MakeCode:

1. Open [https://makecode.microbit.org/](https://makecode.microbit.org/)
2. Click on **New Project**
3. Click on **Extensions** under the gearwheel menu
4. Search for the repository URL and import

---

## Key Concepts Glossary

| Term | What It Means |
|------|---------------|
| **Throttle** | How much power goes to all motors (0% = off, 100% = full power) |
| **Roll** | Tilting left or right |
| **Pitch** | Tilting forward or backward |
| **Yaw** | Spinning left or right |
| **IMU** | Inertial Measurement Unit — a chip with a gyroscope and accelerometer |
| **Gyroscope** | Measures rotation speed (how fast you're spinning) |
| **Accelerometer** | Measures gravity direction (which way is down) |
| **Complementary filter** | Blends gyroscope and accelerometer readings together for a stable angle |
| **PID** | Proportional-Integral-Derivative — a control algorithm that corrects errors |
| **I2C** | A two-wire communication system that lets chips talk to each other |
| **Arm/Disarm** | Enabling or disabling the motors (safety feature) |
| **Failsafe** | Automatic landing when the radio signal is lost |
| **Deadband** | A small zone around the joystick center where tiny movements are ignored |
| **Expo curve** | Makes the joystick less sensitive near the center for smoother control |
| **PWM** | Pulse Width Modulation — a way to control motor speed by switching power on and off very fast |

#### Metadata (used for search, rendering)

* for PXT/microbit
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
