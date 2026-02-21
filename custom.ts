// ============================================================================
// AIRBIT HARDWARE LIBRARY
// ============================================================================
//
// This file talks to the actual hardware chips on the drone:
//
// 1. GYROSCOPE (IMU) - a tiny sensor that measures how the drone is tilted
//    and how fast it is rotating. Like the inner ear that helps you balance.
//
// 2. MOTOR CONTROLLER (PCA9685) - a chip that controls the speed of all 4
//    motors. We tell it a number 0-255 and it sends the right amount of
//    power to each motor.
//
// 3. BAROMETER - measures air pressure to estimate altitude (height).
//
// The micro:bit talks to these chips using I2C, which is like a two-wire
// telephone line. Each chip has its own "phone number" (address).
//
// This file also contains the PID STABILIZATION algorithm - the math that
// keeps the drone level in the air by adjusting motor speeds.
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
            BARO_ADDRESS,
            BARO_CMD_SOFT_RESET,
            NumberFormat.UInt16BE,
            true
        )
        basic.pause(10)
        pins.i2cWriteNumber(
            BARO_ADDRESS,
            BARO_CMD_READ_ID,
            NumberFormat.UInt16BE,
            true
        )
        BARO_return = pins.i2cReadNumber(BARO_ADDRESS, NumberFormat.UInt16LE, true)
        if (!BARO_return) {
            basic.showString("No Baro", 50)
            return
        }
        basic.showString("B")
    }


    // Reset all PID memory to zero — like clearing the slate.
    // Called when the drone is disarmed so old corrections don't carry over.
    //% blockID=airbit_clean_reg
    //% block="Reset Stabilization"
    //% group='Control'
    export function resetPidState() {
        rollDiff = 0
        pitchDiff = 0
        lastRollDiff = 0
        lastPitchDiff = 0
        lastYawDiff = 0
        rollIdiff = 0
        pitchIdiff = 0
        yawIdiff = 0
        yawDiff = 0
        yawDdiff = 0
        pitchDdiff = 0
        rollDdiff = 0
        imuYaw = 0
        gyroZdelta = 0
        yaw = 0
        rollCorrection = 0
        pitchCorrection = 0

    }

    // Get the battery charge as a percentage (0% = empty, 100% = full)
    //% blockID=airbit_battery_level
    //% block="Battery Level"
    //% group='Battery management'
    export function batteryLevel() {
        batteryCalculation()
        return Math.map(batterymVoltSmooth, BATTERY_VOLTAGE_MIN, BATTERY_VOLTAGE_MAX, 0, 100)
    }


    // Read the battery voltage and smooth it out.
    // Smoothing works like a running average — it mixes 10% of the new reading
    // with 90% of the old one. This prevents the number from jumping around.
    //% blockID=airbit_battery_calculation
    //% block="Battery Calculation"
    //% group='Battery management'
    export function batteryCalculation() {
        batterymVoltSmooth = Math.round(pins.analogReadPin(AnalogPin.P0) * BATTERY_FACTOR * BATTERY_SMOOTHING_NEW + batterymVoltSmooth * BATTERY_SMOOTHING_OLD)

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
            PCA_ADDRESS,
            num,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(PCA_ADDRESS, NumberFormat.UInt8BE, false)
    }



    // Calculate the drone's tilt angles from the raw sensor data.
    // Uses a "complementary filter" — it blends two sources:
    //   - The GYROSCOPE is accurate for fast changes (99% weight)
    //   - The ACCELEROMETER is accurate over time (1% weight)
    // Together they give a reliable angle even during vibration.
    //% blockID=airbit_calculate_angles
    //% block="Calculate Angles"
    //% group='Control'
    export function calculateAngles() {
        looptime = input.runningTime() - oldTime
        oldTime = input.runningTime()
        accPitch = (-RAD_TO_DEG * Math.atan2(accY, accZ)) - accPitchOffset
        accRoll = (-RAD_TO_DEG * Math.atan2(accX, accZ)) - accRollOffset
        // Degrees away from desired angle
        gyroXdelta = (gyroX - gyroXcalibration) * looptime * -GYRO_SCALE_FACTOR
        gyroYdelta = (gyroY - gyroYcalibration) * looptime * GYRO_SCALE_FACTOR
        gyroZdelta = (gyroZ - gyroZcalibration) * looptime * -GYRO_SCALE_FACTOR
        imuRoll = (gyroYdelta + imuRoll) * COMPLEMENTARY_GYRO_WEIGHT + accRoll * COMPLEMENTARY_ACC_WEIGHT
        imuPitch = (gyroXdelta + imuPitch) * COMPLEMENTARY_GYRO_WEIGHT + accPitch * COMPLEMENTARY_ACC_WEIGHT
        imuYaw = gyroZdelta + imuYaw
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



    // Send speed values (0-255) to each of the 4 motors via the motor controller chip
    //% blockID=airbit_motor_speed
    //% block="Set Motor Speeds $m0 $m1 $m2 $m3"
    //% m0.min=0 m0.max=255
    //% m1.min=0 m1.max=255
    //% m2.min=0 m2.max=255
    //% m3.min=0 m3.max=255

    //% group='Control'

    export function setMotorSpeeds(m0: number, m1: number, m2: number, m3: number) {
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            PCA_PWM0 << 8 | m3,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            PCA_PWM1 << 8 | m2,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            PCA_PWM2 << 8 | m1,
            NumberFormat.UInt16BE,
            false
        )
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            PCA_PWM3 << 8 | m0,
            NumberFormat.UInt16BE,
            false
        )
    }


    // Wake up the gyroscope/accelerometer sensor and configure it.
    // Shows "G" on the screen if the sensor is found, "NG" if not.
    //% blockID=airbit_start_imu
    //% block="Start Gyroscope"
    //% group='Control'

    // Helper: write a single value to a gyroscope register over I2C
    function writeImuRegister(register: number, value: number) {
        pins.i2cWriteNumber(
            IMU_ADDRESS,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }

    export function startImu() {
        // Full reset chip (H_RESET, internal 20MHz clock)
        writeImuRegister(IMU_PWR_MGMT_1, 0x80)
        basic.pause(500)
        // Read WHO_AM_I register to verify chip is present
        pins.i2cWriteNumber(IMU_ADDRESS, IMU_WHO_AM_I, NumberFormat.UInt8BE, true)
        gyroReturnId = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, false)
        basic.clearScreen()
        if (!(gyroReturnId >> 8 > 0)) {
            basic.showString("NG", 50)
            gyroExists = false
            return
        }
        basic.showString("G")
        gyroExists = true
        writeImuRegister(IMU_PWR_MGMT_1, 0x01)           // Set clock to internal PLL
        writeImuRegister(IMU_SIGNAL_PATH_RESET, 0x07)     // Reset signal paths
        writeImuRegister(IMU_USER_CTRL, 0x00)             // Disable FIFO
        writeImuRegister(IMU_REG_CONFIG, 0)               // Gyro filter: 250 Hz
        writeImuRegister(IMU_ACCEL_CONFIG_2, 5)           // Acc filter: 10.2 Hz
    }



    // Write a value to the motor controller chip over I2C
    //% blockID=airbit_write_pca
    //% block="Write Motor Register"
    //% group='System'
    export function writeMotorRegister(register: number, value: number) {
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            register << 8 | value,
            NumberFormat.UInt16BE,
            false
        )
    }


    // Read raw rotation speed (gyro) and acceleration (acc) data from the sensor.
    // This gives us 6 numbers: gyroX/Y/Z and accX/Y/Z.
    //% blockID=airbit_read_imu
    //% block="Read Gyroscope"
    //% group='Control'
    export function readImuSensors() {
        pins.i2cWriteNumber(
            IMU_ADDRESS,
            IMU_REG_GYRO_XOUT_H,
            NumberFormat.Int8LE,
            true
        )
        gyroX = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, true)
        gyroY = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, true)
        gyroZ = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, false)
        pins.i2cWriteNumber(
            IMU_ADDRESS,
            IMU_REG_ACCEL_XOUT_H,
            NumberFormat.Int8LE,
            true
        )
        accX = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, true)
        accY = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, true)
        accZ = pins.i2cReadNumber(IMU_ADDRESS, NumberFormat.Int16BE, false)
    }



    // Wake up the motor controller chip (PCA9685) and check if it's connected.
    // Shows "M" on the screen if found, "No PCA!" if not.
    //% blockID=airbit_start_pca
    //% block="Start Motors"
    //% group='Control'
    export function startMotorController() {
        writeMotorRegister(PCA_REG_MODE1, PCA_RESET)
        writeMotorRegister(PCA_REG_MODE2, PCA_MODE2_CONFIG)
        writeMotorRegister(PCA_REG_LEDOUT, PCA_LEDOUT_INDIVIDUAL)

        setMotorSpeeds(0, 0, 0, 0)     // Zero out motor speed
        // Self test to see if data reg can be read.
        pins.i2cWriteNumber(
            PCA_ADDRESS,
            PCA_REG_MODE2,
            NumberFormat.UInt8BE,
            true
        )
        mcReturnId = pins.i2cReadNumber(PCA_ADDRESS, NumberFormat.UInt8BE, false)
        basic.clearScreen()
        if (!mcReturnId) {
            basic.showString("No PCA!", 50)
            mcExists = false
            return
        }
        basic.showString("M")
        mcExists = true
    }




    // Calibrate the gyroscope — the drone must be perfectly still!
    // Takes 100 readings and averages them to find the sensor's "zero point".
    // Without this, the drone would think it's always slowly spinning.
    //% blockID=airbit_calibrate_gyro
    //% block="Calibrate Gyroscope"
    //% group='Control'
    export function calibrateGyro() {
        gyroXcalibration = 0
        gyroYcalibration = 0
        gyroZcalibration = 0
        basic.showString("C")
        for (let index = 0; index < GYRO_CALIBRATION_SAMPLES; index++) {
            readImuSensors()
            gyroXcalibration += gyroX
            gyroYcalibration += gyroY
            gyroZcalibration += gyroZ
            basic.pause(5)
        }
        gyroXcalibration = gyroXcalibration / GYRO_CALIBRATION_SAMPLES
        gyroYcalibration = gyroYcalibration / GYRO_CALIBRATION_SAMPLES
        gyroZcalibration = gyroZcalibration / GYRO_CALIBRATION_SAMPLES
        accPitch = -RAD_TO_DEG * Math.atan2(accY, accZ)
        accRoll = -RAD_TO_DEG * Math.atan2(accX, accZ)
        accPitchOffset = accPitch
        accRollOffset = accRoll

        basic.showIcon(IconNames.Yes)
    }




    // PID STABILIZATION — the brain of the drone!
    //
    // This function compares WHERE the pilot wants the drone to be (roll, pitch, yaw)
    // with WHERE the drone actually is (imuRoll, imuPitch, imuYaw).
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
        rollDiff = roll - imuRoll
        pitchDiff = pitch - imuPitch
        yawDiff = yaw - imuYaw

        // Step 2: D (Derivative) — how fast is the error changing?
        rollDdiff = rollDiff - lastRollDiff
        pitchDdiff = pitchDiff - lastPitchDiff
        yawDdiff = yawDiff - lastYawDiff

        // Remember this error for next time (needed to calculate derivative)
        lastRollDiff = rollDiff
        lastPitchDiff = pitchDiff
        lastYawDiff = yawDiff

        // Step 3: I (Integral) — accumulate small persistent errors over time.
        // Only active when flying (throttle high enough) and error is small.
        if (throttle > INTEGRAL_THROTTLE_THRESHOLD) {
            if (rollDiff > -INTEGRAL_RANGE && rollDiff < INTEGRAL_RANGE) {
                rollIdiff += rollDiff
            }
            if (pitchDiff > -INTEGRAL_RANGE && pitchDiff < INTEGRAL_RANGE) {
                pitchIdiff += pitchDiff
            }
        }

        // Limit the integral correction so it doesn't overcorrect
        let rollIcorrection = Math.constrain(rollIdiff * rollPitchI, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        let pitchIcorrection = Math.constrain(pitchIdiff * rollPitchI, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        // Step 4: Combine P + I + D into a total correction for each axis
        rollCorrection = rollDiff * rollPitchP + rollIcorrection + rollDdiff * rollPitchD
        pitchCorrection = pitchDiff * rollPitchP + pitchIcorrection + pitchDdiff * rollPitchD
        yawCorrection = yawDiff * yawP + yawDdiff * yawD
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
            PCA_ADDRESS,
            PCA_REG_MODE1,
            NumberFormat.UInt8BE,
            true
        )
        return pins.i2cReadNumber(PCA_ADDRESS, NumberFormat.UInt8BE, false)
    }


    // ========================================================================
    // CONSTANTS — settings that never change
    // ========================================================================

    // --- IMU (Gyro/Accelerometer) Register Addresses ---
    // These are "mailbox numbers" inside the gyroscope chip
    const IMU_ADDRESS = 104
    const IMU_REG_CONFIG = 1
    const IMU_PWR_MGMT_1 = 107
    const IMU_WHO_AM_I = 117
    const IMU_SIGNAL_PATH_RESET = 105
    const IMU_USER_CTRL = 106
    const IMU_ACCEL_CONFIG_2 = 29
    const IMU_REG_GYRO_XOUT_H = 67
    const IMU_REG_ACCEL_XOUT_H = 59

    // --- IMU Sensor Fusion Constants ---
    // Used by the complementary filter that blends gyro + accelerometer
    const RAD_TO_DEG = 57.295
    const GYRO_SCALE_FACTOR = 0.00000762939
    const COMPLEMENTARY_GYRO_WEIGHT = 0.99
    const COMPLEMENTARY_ACC_WEIGHT = 0.01
    const GYRO_CALIBRATION_SAMPLES = 100

    // --- Motor Controller (PCA9685) Register Addresses ---
    // These are "mailbox numbers" inside the motor controller chip
    const PCA_ADDRESS = 98
    const PCA_REG_MODE1 = 0
    const PCA_REG_MODE2 = 1
    const PCA_REG_LEDOUT = 8
    const PCA_PWM0 = 2
    const PCA_PWM1 = 3
    const PCA_PWM2 = 4
    const PCA_PWM3 = 5
    const PCA_RESET = 128
    const PCA_LEDOUT_INDIVIDUAL = 170
    const PCA_MODE2_CONFIG = 5  // Non-inverted, Totem pole

    // --- Barometer Register Addresses ---
    const BARO_ADDRESS = 99
    const BARO_CMD_SOFT_RESET = 32861
    const BARO_CMD_READ_ID = 61384

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

    // --- IMU Sensor State ---
    let gyroReturnId = 0
    let mcReturnId = 0
    let BARO_return = 0
    let gyroX = 0
    let gyroY = 0
    let gyroZ = 0
    let gyroXdelta = 0
    let gyroYdelta = 0
    let gyroZdelta = 0
    let gyroXcalibration = 0
    let gyroYcalibration = 0
    let gyroZcalibration = 0
    let accX = 0
    let accY = 0
    let accZ = 0
    let accPitch = 0
    let accRoll = 0
    let accPitchOffset = 0
    let accRollOffset = 0
    let looptime = 0
    let oldTime = 0

    // --- PID Calculation State ---
    // These hold the intermediate math values used by the stabilize() function
    let throttleScaled = 0
    let rollCorrection = 0
    let pitchCorrection = 0
    let yawCorrection = 0
    let rollDiff = 0
    let pitchDiff = 0
    let yawDiff = 0
    let lastRollDiff = 0
    let lastPitchDiff = 0
    let lastYawDiff = 0
    let rollDdiff = 0
    let pitchDdiff = 0
    let yawDdiff = 0
    let rollIdiff = 0
    let pitchIdiff = 0
    let yawIdiff = 0





}

