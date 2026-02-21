// ============================================================================
// AIRBIT HARDWARE LIBRARY
// ============================================================================
//
// This file talks to the actual hardware chips on the drone board.
//
// WHAT IS I2C?
// ------------
// The micro:bit needs to talk to several tiny chips on the drone board.
// It does this using I2C (pronounced "I-squared-C"), a communication system
// that works like a shared telephone line with just two wires:
//   - SDA (data wire) — carries the actual messages back and forth
//   - SCL (clock wire) — keeps everyone in sync, like a ticking metronome
// Every chip on the line has its own "phone number" (called an address).
// When the micro:bit wants to talk to a specific chip, it dials that
// address first, then sends or asks for data. Only the chip with that
// address responds — the others stay quiet.
//
// WHAT IS AN IMU (GYROSCOPE + ACCELEROMETER)?
// --------------------------------------------
// An IMU (Inertial Measurement Unit) is a tiny sensor that measures movement.
// Ours contains two sensors in one chip:
//   - GYROSCOPE — measures how fast the drone is spinning (rotation speed).
//     Think of spinning a basketball on your finger — the gyroscope measures
//     how quickly it turns. It is very accurate for quick movements, but
//     over time it slowly "drifts" (like a clock that gains a few seconds
//     each hour).
//   - ACCELEROMETER — measures which direction gravity pulls. When the
//     drone tilts, gravity pulls sideways a little, and the accelerometer
//     detects that tilt. It doesn't drift, but vibrations from the motors
//     make it jittery during flight.
// We combine both sensors together (see calculateAngles) to get a tilt
// measurement that is both accurate and stable.
//
// THE CHIPS ON OUR DRONE:
// -----------------------
// 1. GYROSCOPE (MPU-6050 IMU) — I2C address 104
//    Measures tilt and rotation. Like the inner ear that helps you balance.
//
// 2. MOTOR CONTROLLER (PCA9685) — I2C address 98
//    Controls the speed of all 4 motors. We tell it a number 0-255 and it
//    sends the right amount of power to each motor.
//
// 3. BAROMETER — I2C address 99
//    Measures air pressure to estimate altitude (height above ground).
//
// This file also contains the PID STABILIZATION algorithm — the math that
// keeps the drone level in the air by adjusting motor speeds hundreds of
// times per second.
// ============================================================================

//% weight=100 color=#0fbc11 icon=""
namespace airbit {


    // Draw a vertical bar on the LED screen — used for throttle and battery displays
    //% blockID=airbit_smart_bar
    //% block="Smart Bar $x $amount"
    //% group='Screen'
    //% x.min = 0 x.max=4
    //% amount.min = 0 amount.max = 100

    export function smartBar(x: number, amount: number) {
        for (let index = 0; index <= amount / 20; index++) {
            led.plot(x, 4 - index)
        }
        led.plotBrightness(x, 4 - Math.floor(amount / 20), 12.75 * (amount % 20))
    }

    // Wake up the barometer (air pressure sensor) and check if it's connected
    //% blockID=airbit_start_baro
    //% block="Start Barometer"
    //% group='Control'
    export function baroStart() {
        // Soft reset
        pins.i2cWriteNumber(
            BAROMETER_ADDRESS,
            BAROMETER_CMD_SOFT_RESET,
            NumberFormat.UInt16BE,
            true
        )
        basic.pause(10)
        pins.i2cWriteNumber(
            BAROMETER_ADDRESS,
            BAROMETER_CMD_READ_ID,
            NumberFormat.UInt16BE,
            true
        )
        barometerReturnId = pins.i2cReadNumber(BAROMETER_ADDRESS, NumberFormat.UInt16LE, true)
        if (!barometerReturnId) {
            basic.showString("No Baro", 50)
            return
        }
        basic.showString("B")
    }


    // Reset all PID memory to zero — like clearing the slate.
    // WHY? The PID algorithm remembers past errors (integral) and how fast things
    // were changing (derivative). If we don't clear these when the drone is
    // disarmed, old corrections from the last flight would kick in the moment
    // we take off again — potentially causing a sudden jerk or flip.
    //% blockID=airbit_clean_reg
    //% block="Reset Stabilization"
    //% group='Control'
    export function resetPidState() {
        rollError = 0
        pitchError = 0
        lastRollError = 0
        lastPitchError = 0
        lastYawError = 0
        rollIntegral = 0
        pitchIntegral = 0
        yawIntegral = 0
        yawError = 0
        yawDerivative = 0
        pitchDerivative = 0
        rollDerivative = 0
        measuredYaw = 0
        gyroscopeDeltaZ = 0
        yaw = 0
        rollCorrection = 0
        pitchCorrection = 0

    }

    // Get the battery charge as a percentage (0% = empty, 100% = full).
    // WHY? The battery reports its charge as a voltage (3400-4200 millivolts),
    // but that's not intuitive. We convert it to 0-100% so the pilot can quickly
    // understand how much flight time is left — just like a phone battery icon.
    //% blockID=airbit_battery_level
    //% block="Battery Level"
    //% group='Battery management'
    export function batteryLevel() {
        batteryCalculation()
        return Math.map(batteryMillivoltsSmoothed, BATTERY_VOLTAGE_MIN, BATTERY_VOLTAGE_MAX, 0, 100)
    }


    // Read the battery voltage and smooth it out.
    // WHY SMOOTH? The raw battery reading jumps up and down because of electrical
    // noise (like static on a radio). If we used it directly, the battery display
    // would flicker constantly. Smoothing mixes 10% of the new reading with 90%
    // of the old one — like slowly stirring new paint into an existing color.
    // This gives a stable number that still tracks real changes over time.
    //% blockID=airbit_battery_calculation
    //% block="Battery Calculation"
    //% group='Battery management'
    export function batteryCalculation() {
        batteryMillivoltsSmoothed = Math.round(pins.analogReadPin(AnalogPin.P0) * BATTERY_FACTOR * BATTERY_SMOOTHING_NEW + batteryMillivoltsSmoothed * BATTERY_SMOOTHING_OLD)

    }

    // Read the raw battery voltage in millivolts (no smoothing)
    //% blockID=airbit_battery_calculation_simple
    //% block="Battery Millivolts"
    //% group='Battery management'
    export function batteryMillivolts() {
        return Math.round(pins.analogReadPin(AnalogPin.P0) * BATTERY_FACTOR)

    }




    // Read a value from the motor controller chip over I2C
    //% blockID=airbit_read_pca
    //% block="Read Motor Register"
    //% group='System'
    export function readMotorRegister(num: number) {
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            num,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(MOTOR_CONTROLLER_ADDRESS, NumberFormat.UInt8BE, false)
    }



    // Calculate the drone's tilt angles from the raw sensor data.
    //
    // WHY DO WE NEED THIS? The gyroscope and accelerometer give us raw numbers,
    // but we need to know the actual tilt angle in degrees (like "tilted 5° to
    // the right"). Getting a good angle is tricky because each sensor has a flaw:
    //   - The GYROSCOPE tells us how fast we're rotating, not the angle itself.
    //     We calculate the angle by adding up tiny rotations over time, but small
    //     errors accumulate — like a pedometer that slowly over-counts your steps.
    //   - The ACCELEROMETER can measure the tilt angle directly from gravity, but
    //     motor vibrations make it jittery — like trying to read a book on a bus.
    //
    // THE SOLUTION: A "complementary filter" blends both sensors together:
    //   angle = 99% × gyroscope angle + 1% × accelerometer angle
    // The gyroscope handles quick movements (99% trust), while the accelerometer
    // gently corrects the drift over time (1% nudge). It's like navigating with
    // a compass that wobbles (accelerometer) and a step-counter that drifts
    // (gyroscope) — together, they keep you on track.
    //% blockID=airbit_calculate_angles
    //% block="Calculate Angles"
    //% group='Control'
    export function calculateAngles() {
        looptime = input.runningTime() - oldTime
        oldTime = input.runningTime()
        accelerometerPitch = (-RAD_TO_DEG * Math.atan2(accelerometerY, accelerometerZ)) - accelerometerPitchOffset
        accelerometerRoll = (-RAD_TO_DEG * Math.atan2(accelerometerX, accelerometerZ)) - accelerometerRollOffset
        // Degrees away from desired angle
        gyroscopeDeltaX = (gyroscopeX - gyroscopeCalibrationX) * looptime * -GYRO_SCALE_FACTOR
        gyroscopeDeltaY = (gyroscopeY - gyroscopeCalibrationY) * looptime * GYRO_SCALE_FACTOR
        gyroscopeDeltaZ = (gyroscopeZ - gyroscopeCalibrationZ) * looptime * -GYRO_SCALE_FACTOR
        measuredRoll = (gyroscopeDeltaY + measuredRoll) * COMPLEMENTARY_GYRO_WEIGHT + accelerometerRoll * COMPLEMENTARY_ACC_WEIGHT
        measuredPitch = (gyroscopeDeltaX + measuredPitch) * COMPLEMENTARY_GYRO_WEIGHT + accelerometerPitch * COMPLEMENTARY_ACC_WEIGHT
        measuredYaw = gyroscopeDeltaZ + measuredYaw
    }

    // Animate a dot spinning in a circle on the LED screen (used in motor test)
    //% blockID=airbit_rotation_dot
    //% block="Rotation dot $xPos $yPos $radius $speed"
    //% xPos.min=0 xPos.max=4 xPos.dfl=2
    //% yPos.min=0 yPos.max=4 yPos.dfl=2
    //% radius.min=1 radius.max=4 radius.dfl=2
    //% speed.min=-100 speed.max=100 speed.dfl=10
    //% group='Screen'

    export function rotateDot(xPos: number, yPos: number, radius: number, speed: number) {
        led.plot(xPos + 0.5 + (radius+0.5) * Math.cos(input.runningTime() / 10000 * 6.283 * speed), yPos + 0.5 + (radius+0.5) * Math.sin(input.runningTime() / 10000 * 6.283 * speed))
    }



    // Send speed values (0-255) to each of the 4 motors via the motor controller chip.
    // Each I2C message packs two pieces of info into one 16-bit number:
    //   - The register address (which motor) in the high byte
    //   - The speed value (0-255) in the low byte
    // The "<<8 |" trick shifts the register left by 8 bits and combines it with
    // the speed, like writing a street address and apartment number on one line.
    //% blockID=airbit_motor_speed
    //% block="Set Motor Speeds $m0 $m1 $m2 $m3"
    //% m0.min=0 m0.max=255
    //% m1.min=0 m1.max=255
    //% m2.min=0 m2.max=255
    //% m3.min=0 m3.max=255

    //% group='Control'

    export function setMotorSpeeds(m0: number, m1: number, m2: number, m3: number) {
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_PWM0 << 8 | m3,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_PWM1 << 8 | m2,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_PWM2 << 8 | m1,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_PWM3 << 8 | m0,
            NumberFormat.UInt16BE,
            false
        )
    }


    // Wake up the gyroscope/accelerometer sensor (IMU) and configure it.
    // WHY? When the chip first powers on, it's in sleep mode to save energy.
    // We need to: 1) wake it up, 2) verify it's actually there by reading its
    // ID (like asking "who are you?"), and 3) configure the filters that smooth
    // out the raw sensor data. Shows "G" on the screen if found, "NG" if not.
    //% blockID=airbit_start_imu
    //% block="Start Gyroscope"
    //% group='Control'

    // Helper: write a single value to a gyroscope register over I2C
    function writeGyroscopeRegister(register: number, value: number) {
        pins.i2cWriteNumber(
            GYROSCOPE_ADDRESS,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }

    export function startImu() {
        // Full reset chip (H_RESET, internal 20MHz clock)
        writeGyroscopeRegister(GYROSCOPE_PWR_MGMT_1, 0x80)
        basic.pause(500)
        // Read WHO_AM_I register to verify chip is present
        pins.i2cWriteNumber(GYROSCOPE_ADDRESS, GYROSCOPE_WHO_AM_I, NumberFormat.UInt8BE, true)
        gyroscopeReturnId = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, false)
        basic.clearScreen()
        if (!(gyroscopeReturnId >> 8 > 0)) {
            basic.showString("NG", 50)
            gyroExists = false
            return
        }
        basic.showString("G")
        gyroExists = true
        writeGyroscopeRegister(GYROSCOPE_PWR_MGMT_1, 0x01)           // Set clock to internal PLL
        writeGyroscopeRegister(GYROSCOPE_SIGNAL_PATH_RESET, 0x07)     // Reset signal paths
        writeGyroscopeRegister(GYROSCOPE_USER_CTRL, 0x00)             // Disable FIFO
        writeGyroscopeRegister(GYROSCOPE_REG_CONFIG, 0)               // Gyro filter: 250 Hz
        writeGyroscopeRegister(GYROSCOPE_ACCEL_CONFIG_2, 5)           // Acc filter: 10.2 Hz
    }



    // Write a value to the motor controller chip over I2C
    //% blockID=airbit_write_pca
    //% block="Write Motor Register"
    //% group='System'
    export function writeMotorRegister(register: number, value: number) {
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }


    // Read raw rotation speed and acceleration data from the IMU sensor over I2C.
    // This gives us 6 numbers: gyroscopeX/Y/Z (how fast we're spinning on each
    // axis) and accelerometerX/Y/Z (which direction gravity is pulling).
    // WHY? These raw numbers are the drone's sense of balance — without them,
    // the drone has no idea which way is up. We read them hundreds of times per
    // second so the drone can react instantly to any tilt or gust of wind.
    //% blockID=airbit_read_imu
    //% block="Read Gyroscope"
    //% group='Control'
    export function readImuSensors() {
        pins.i2cWriteNumber(
            GYROSCOPE_ADDRESS,
            GYROSCOPE_REG_GYRO_XOUT_H,
            NumberFormat.Int8LE,
            true
        )
        gyroscopeX = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, true)
        gyroscopeY = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, true)
        gyroscopeZ = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, false)
        pins.i2cWriteNumber(
            GYROSCOPE_ADDRESS,
            GYROSCOPE_REG_ACCEL_XOUT_H,
            NumberFormat.Int8LE,
            true
        )
        accelerometerX = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, true)
        accelerometerY = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, true)
        accelerometerZ = pins.i2cReadNumber(GYROSCOPE_ADDRESS, NumberFormat.Int16BE, false)
    }



    // Wake up the motor controller chip (PCA9685) and check if it's connected.
    // WHY? The micro:bit can't power the motors directly — they need too much
    // current. The motor controller chip acts as a middleman: we send it small
    // I2C messages saying "motor 1 at speed 200", and it handles the heavy
    // electrical work. We reset it, configure its output mode, then verify it
    // responds by reading back a register. Shows "M" if found, "No PCA!" if not.
    //% blockID=airbit_start_pca
    //% block="Start Motors"
    //% group='Control'
    export function startMotorController() {
        writeMotorRegister(MOTOR_CONTROLLER_REG_MODE1, MOTOR_CONTROLLER_RESET)
        writeMotorRegister(MOTOR_CONTROLLER_REG_MODE2, MOTOR_CONTROLLER_MODE2_CONFIG)
        writeMotorRegister(MOTOR_CONTROLLER_REG_LEDOUT, MOTOR_CONTROLLER_LEDOUT_INDIVIDUAL)

        setMotorSpeeds(0, 0, 0, 0)     // Zero out motor speed
        // Self test to see if data reg can be read.
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_REG_MODE2,
            NumberFormat.UInt8BE,
            true
        )
        motorControllerReturnId = pins.i2cReadNumber(MOTOR_CONTROLLER_ADDRESS, NumberFormat.UInt8BE, false)
        basic.clearScreen()
        if (!motorControllerReturnId) {
            basic.showString("No PCA!", 50)
            motorControllerExists = false
            return
        }
        basic.showString("M")
        motorControllerExists = true
    }




    // Calibrate the gyroscope — the drone must be perfectly still!
    // WHY? Even when sitting still, the gyroscope doesn't read exactly zero.
    // Every sensor has a small built-in error called "bias" — like a bathroom
    // scale that reads 0.3 kg with nothing on it. If we don't correct for this,
    // the drone would think it's slowly spinning and try to fight a rotation
    // that isn't there. Calibration takes 100 readings while the drone is still,
    // averages them to find this bias, and subtracts it from all future readings.
    // It also records the current tilt as "level" using the accelerometer.
    //% blockID=airbit_calibrate_gyro
    //% block="Calibrate Gyroscope"
    //% group='Control'
    export function calibrateGyro() {
        gyroscopeCalibrationX = 0
        gyroscopeCalibrationY = 0
        gyroscopeCalibrationZ = 0
        basic.showString("C")
        for (let index = 0; index < GYRO_CALIBRATION_SAMPLES; index++) {
            readImuSensors()
            gyroscopeCalibrationX += gyroscopeX
            gyroscopeCalibrationY += gyroscopeY
            gyroscopeCalibrationZ += gyroscopeZ
            basic.pause(5)
        }
        gyroscopeCalibrationX = gyroscopeCalibrationX / GYRO_CALIBRATION_SAMPLES
        gyroscopeCalibrationY = gyroscopeCalibrationY / GYRO_CALIBRATION_SAMPLES
        gyroscopeCalibrationZ = gyroscopeCalibrationZ / GYRO_CALIBRATION_SAMPLES
        accelerometerPitch = -RAD_TO_DEG * Math.atan2(accelerometerY, accelerometerZ)
        accelerometerRoll = -RAD_TO_DEG * Math.atan2(accelerometerX, accelerometerZ)
        accelerometerPitchOffset = accelerometerPitch
        accelerometerRollOffset = accelerometerRoll

        basic.showIcon(IconNames.Yes)
    }




    // Accumulate the integral (I) term — a running total of small errors over time.
    // WHY? Imagine the drone keeps drifting slightly left. The P correction pushes
    // back, but not quite enough. The integral slowly adds up these small leftover
    // errors and pushes harder and harder until the drift stops — like leaning more
    // and more into a wind until you stop being pushed sideways.
    // We only accumulate when actually flying (throttle up) and when the error is
    // small. Large errors mean the drone is doing a big maneuver on purpose, and
    // we don't want the integral to "wind up" and overcorrect afterwards.
    function accumulateIntegral() {
        if (throttle <= INTEGRAL_THROTTLE_THRESHOLD) return
        if (rollError > -INTEGRAL_RANGE && rollError < INTEGRAL_RANGE) {
            rollIntegral += rollError
        }
        if (pitchError > -INTEGRAL_RANGE && pitchError < INTEGRAL_RANGE) {
            pitchIntegral += pitchError
        }
    }

    // PID STABILIZATION — the brain of the drone!
    //
    // This function compares WHERE the pilot wants the drone to be (roll, pitch, yaw)
    // with WHERE the drone actually is (measuredRoll, measuredPitch, measuredYaw).
    // The difference is the "error". PID calculates how to correct it:
    //
    //   P (Proportional): correct based on how far off we are RIGHT NOW
    //   I (Integral):     correct based on errors that have been BUILDING UP over time
    //   D (Derivative):   correct based on how fast the error is CHANGING
    //
    // The result is 4 motor speeds. To tilt right, speed up the left motors
    // and slow down the right motors. To spin, change diagonal motor pairs.
    //
    //   motorA = throttle + roll + pitch + yaw   (back-left)
    //   motorB = throttle + roll - pitch - yaw   (front-left)
    //   motorC = throttle - roll + pitch - yaw   (back-right)
    //   motorD = throttle - roll - pitch + yaw   (front-right)
    //
    //% blockID=airbit_stabilise_pid
    //% block="Stabilize Drone"
    //% group='Control'
    export function stabilize() {
        // Step 1: Calculate the error (difference between desired and actual angle)
        rollError = roll - measuredRoll
        pitchError = pitch - measuredPitch
        yawError = yaw - measuredYaw

        // Step 2: D (Derivative) — how fast is the error changing?
        rollDerivative = rollError - lastRollError
        pitchDerivative = pitchError - lastPitchError
        yawDerivative = yawError - lastYawError

        // Remember this error for next time (needed to calculate derivative)
        lastRollError = rollError
        lastPitchError = pitchError
        lastYawError = yawError

        // Step 3: I (Integral) — accumulate small persistent errors over time.
        // Only active when flying (throttle high enough) and error is small.
        accumulateIntegral()

        // Limit the integral correction so it doesn't overcorrect
        let rollIcorrection = Math.constrain(rollIntegral * rollPitchI, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        let pitchIcorrection = Math.constrain(pitchIntegral * rollPitchI, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        // Step 4: Combine P + I + D into a total correction for each axis
        rollCorrection = rollError * rollPitchP + rollIcorrection + rollDerivative * rollPitchD
        pitchCorrection = pitchError * rollPitchP + pitchIcorrection + pitchDerivative * rollPitchD
        yawCorrection = yawError * yawP + yawDerivative * yawD
        yawCorrection = Math.constrain(yawCorrection, -YAW_CORRECTION_LIMIT, YAW_CORRECTION_LIMIT)

        // Step 5: Convert throttle percentage (0-100) to motor range (0-255)
        throttleScaled = throttle * THROTTLE_SCALE

        // Step 6: Mix throttle with corrections to get each motor's speed.
        // Adding/subtracting corrections makes the drone tilt in the right direction.
        motorA = Math.round(throttleScaled + rollCorrection + pitchCorrection + yawCorrection)
        motorB = Math.round(throttleScaled + rollCorrection - pitchCorrection - yawCorrection)
        motorC = Math.round(throttleScaled - rollCorrection + pitchCorrection - yawCorrection)
        motorD = Math.round(throttleScaled - rollCorrection - pitchCorrection + yawCorrection)

        // Safety: keep motor speeds within valid range (0-255)
        motorA = Math.constrain(motorA, 0, 255)
        motorB = Math.constrain(motorB, 0, 255)
        motorC = Math.constrain(motorC, 0, 255)
        motorD = Math.constrain(motorD, 0, 255)
    }


    // Read the current operating mode of the motor controller chip
    export function readMotorControllerMode() {
        pins.i2cWriteNumber(
            MOTOR_CONTROLLER_ADDRESS,
            MOTOR_CONTROLLER_REG_MODE1,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(MOTOR_CONTROLLER_ADDRESS, NumberFormat.UInt8BE, false)
    }


    // ========================================================================
    // CONSTANTS — settings that never change
    // ========================================================================

    // --- Gyroscope/Accelerometer Register Addresses ---
    // These are "mailbox numbers" inside the gyroscope chip
    const GYROSCOPE_ADDRESS = 104
    const GYROSCOPE_REG_CONFIG = 1
    const GYROSCOPE_PWR_MGMT_1 = 107
    const GYROSCOPE_WHO_AM_I = 117
    const GYROSCOPE_SIGNAL_PATH_RESET = 105
    const GYROSCOPE_USER_CTRL = 106
    const GYROSCOPE_ACCEL_CONFIG_2 = 29
    const GYROSCOPE_REG_GYRO_XOUT_H = 67
    const GYROSCOPE_REG_ACCEL_XOUT_H = 59

    // --- Gyroscope Sensor Fusion Constants ---
    // Used by the complementary filter that blends gyro + accelerometer
    const RAD_TO_DEG = 57.295
    const GYRO_SCALE_FACTOR = 0.00000762939
    const COMPLEMENTARY_GYRO_WEIGHT = 0.99
    const COMPLEMENTARY_ACC_WEIGHT = 0.01
    const GYRO_CALIBRATION_SAMPLES = 100

    // --- Motor Controller (PCA9685) Register Addresses ---
    // These are "mailbox numbers" inside the motor controller chip
    const MOTOR_CONTROLLER_ADDRESS = 98
    const MOTOR_CONTROLLER_REG_MODE1 = 0
    const MOTOR_CONTROLLER_REG_MODE2 = 1
    const MOTOR_CONTROLLER_REG_LEDOUT = 8
    const MOTOR_CONTROLLER_PWM0 = 2
    const MOTOR_CONTROLLER_PWM1 = 3
    const MOTOR_CONTROLLER_PWM2 = 4
    const MOTOR_CONTROLLER_PWM3 = 5
    const MOTOR_CONTROLLER_RESET = 128
    const MOTOR_CONTROLLER_LEDOUT_INDIVIDUAL = 170
    const MOTOR_CONTROLLER_MODE2_CONFIG = 5  // Non-inverted, Totem pole

    // --- Barometer Register Addresses ---
    const BAROMETER_ADDRESS = 99
    const BAROMETER_CMD_SOFT_RESET = 32861
    const BAROMETER_CMD_READ_ID = 61384

    // --- Battery Constants ---
    const BATTERY_FACTOR = 5.94   // Converts analog pin reading to millivolts
    const BATTERY_VOLTAGE_MIN = 3400
    const BATTERY_VOLTAGE_MAX = 4200
    const BATTERY_SMOOTHING_NEW = 0.1
    const BATTERY_SMOOTHING_OLD = 0.9

    // --- PID Constants ---
    const INTEGRAL_RANGE = 5
    const INTEGRAL_LIMIT = 4
    const YAW_CORRECTION_LIMIT = 50
    const INTEGRAL_THROTTLE_THRESHOLD = 50
    const THROTTLE_SCALE = 2.55

    // ========================================================================
    // VARIABLES — values that change while the drone is running
    // ========================================================================

    // --- Gyroscope Sensor State ---
    let gyroscopeReturnId = 0
    let motorControllerReturnId = 0
    let barometerReturnId = 0
    let gyroscopeX = 0
    let gyroscopeY = 0
    let gyroscopeZ = 0
    let gyroscopeDeltaX = 0
    let gyroscopeDeltaY = 0
    let gyroscopeDeltaZ = 0
    let gyroscopeCalibrationX = 0
    let gyroscopeCalibrationY = 0
    let gyroscopeCalibrationZ = 0
    let accelerometerX = 0
    let accelerometerY = 0
    let accelerometerZ = 0
    let accelerometerPitch = 0
    let accelerometerRoll = 0
    let accelerometerPitchOffset = 0
    let accelerometerRollOffset = 0
    let looptime = 0
    let oldTime = 0

    // --- PID Calculation State ---
    // These hold the intermediate math values used by the stabilize() function
    let throttleScaled = 0
    let rollCorrection = 0
    let pitchCorrection = 0
    let yawCorrection = 0
    let rollError = 0
    let pitchError = 0
    let yawError = 0
    let lastRollError = 0
    let lastPitchError = 0
    let lastYawError = 0
    let rollDerivative = 0
    let pitchDerivative = 0
    let yawDerivative = 0
    let rollIntegral = 0
    let pitchIntegral = 0
    let yawIntegral = 0





}

