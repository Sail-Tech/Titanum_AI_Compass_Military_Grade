# Titanum_AI_Compass_Military_Grade - Precision9 N2K - V10.0 
HIGH-PRECISION NAVIGATION SYSTEM (PRECISION-9 / EVO1 EMULATION)

Installation and Operation Manual: N2K Precision Compass V10.0 MIL-SPEC

This system transforms an ESP32, an ICM-20948 inertial sensor, and a BME280 environmental sensor into a professional-grade 9-axis Inertial Measurement Unit (IMU) and Weather Station. Fully compatible with NMEA 2000 and SeaTalk NG networks, this equipment rivals top-tier commercial sensors (such as the Simrad Precision-9 or Raymarine EV-1) thanks to its advanced fusion algorithms and continuous background learning.

1. Introduction and Key Features

The V10.0 MIL-SPEC version introduces technologies typically reserved for aerospace and commercial shipping systems, allowing the sensor to autonomously "understand" and adapt to the vessel's dynamics.

Continuous Magnetic Calibration (Auto-Learning): Powered by a Spherical Gradient Descent algorithm, the sensor continuously learns the vessel's magnetic environment. If you move batteries, install new speakers, or place a metallic toolbox near the sensor, it adapts silently and gradually (preventing sudden heading jumps), maintaining a ±2° heading accuracy.

Magnetic Sanity Check (Anti-Anomaly): The learning algorithm features a strict safety threshold. If exposed to massive, sudden magnetic distortion (e.g., >150 µT, like placing a heavy magnet directly on the device), it refuses to learn the "insane" data, throwing a MAG ANOMALY! warning to protect the core calibration.

Mounting Orientation Matrix: The sensor no longer needs to be mounted perfectly flat. You can mount it upside-down on the ceiling or vertically on a bulkhead; the software instantly remaps the 9-axis matrix to the new physical orientation.

Autopilot Hydraulic Protection (RoT Low-Pass Filter): An Exponential Moving Average (EMA) filter smoothens the Rate of Turn (RoT) data before broadcasting. This provides a silky-smooth rotation curve, preventing autopilot "stuttering" and saving the hydraulic pump from overheating and premature wear.

Bluetooth Low Energy (BLE): A universal wireless interface, 100% compatible with iPhone (iOS) and Android without classic pairing restrictions. Configure the system instantly from your phone anywhere on the deck using optimized 512-byte MTU packets.

9-DOF Fusion (Mahony AHRS): Simultaneous processing of the Gyroscope, Accelerometer, and Magnetometer in a single 3D matrix. The result is a heading immune to roll, pitch, and high-G centrifugal forces during tight turns.

Dynamic Heave Calculation: Uses Bernd Cirotzki's double-integration logic coupled with Aranovskiy's frequency analysis to accurately predict the vessel's vertical displacement (Heave) in the waves.

Embedded Environmental Station (BME280): Broadcasts Temperature, Barometric Pressure, and Humidity to the N2K network with configurable source tagging (e.g., Engine Room, Outside, Cabin).

2. Algorithmic Architecture (The Brains)

The true value of this compass lies in its complex mathematical stack running at 100Hz on the ESP32's coprocessor:

Mahony AHRS (9-DOF) with Quaternions: Merges gyro, accel, and mag data into a single 3D matrix, avoiding Gimbal Lock. Features Centrifugal Compensation, which temporarily reduces accelerometer trust ($K_p$ gain) during high-G maneuvers to prevent false horizon tilts.

Spherical Gradient Descent (Auto-Mag): The engine behind continuous learning. It evaluates the 3D magnetic radius in real-time. If the boat turns and detects a structural anomaly, it applies a microscopic correction ($\mu = 0.00005$) to "pull" the magnetic bias center. This adaptation takes minutes, making it completely imperceptible to the autopilot.

Bernd Cirotzki's Heave Logic: An advanced empirical double-integration algorithm. It rotates the boat's Z-acceleration vector to the absolute Earth frame and applies a dynamic High-Pass/Tendency filter to output precise Heave in meters.

Aranovskiy Frequency Estimator: An adaptive, non-linear filter that extracts true wave frequency directly from vertical acceleration noise, bypassing heavy Fast Fourier Transforms (FFT).

Lemire's Min-Max Algorithm: An $O(1)$ sliding window algorithm that records heave extremes, calculating Wave Height with maximum memory efficiency.

Trochoidal Model (Bareboat Math): Assuming open-ocean waves follow a trochoidal profile, this derives Wave Length and Period based on measured frequency and displacement.

Dynamic 1D Kalman Filter: Applies final smoothing to the Heading. The measurement noise factor ($R$) autonomously adapts based on the boat's Length and Tonnage (input by the user).

3. Use Cases

Due to the high update rate (20Hz Heading, 10Hz Attitude/Heave) and algorithmic precision, this equipment enables:

High-Performance Autopilot: Delivers extremely fast and stable Heading and Rate of Turn (RoT) data, ensuring the autopilot tracks straight without zig-zagging, even at high speeds or in following seas.

Radar Overlay (MARPA): Zero-latency heading accuracy allows perfect radar image overlay on the chartplotter.

Corrosion Prevention (Engine Room): If installed in the engine room, the BME280 monitors relative humidity, sending data to your Multi-Function Display (MFD) to warn of condensation/corrosion risks.

Swell/Wave Analysis: Heave, Wave Frequency, and Period calculations provide exact sea state data, often required by advanced sonar modules for heave compensation.

4. Hardware and Physical Installation

Required Components

Microcontroller: ESP32 Development Board (30 or 38 pins).

IMU Module: ICM-20948 Sensor (I2C Interface).

Environmental Module: BME280 Sensor (Pressure, Temp, Humidity).

CAN Module: 3.3V CAN Transceiver (e.g., SN65HVD230, ISO1050, or MCP2562 powered via 3.3V).

Wiring Schematic (Pinout)

Module

Module Pin

ESP32 Pin

Notes

ICM-20948

VCC / VIN

3.3V

CRITICAL: Do NOT connect to 5V.

ICM-20948
GND


ICM-20948
SDA
GPIO 16

Shared with BME280
ICM-20948
SCL
GPIO 17
Shared with BME280
VCC / VIN
3.3V

CRITICAL: Do NOT connect to 5V.
CAN Transceiver
TX / CTX
GPIO 32

Connects to internal CAN controller
CAN Transceiver
RX / CRX
GPIO 34

Connects to internal CAN controller

Installation Tips

Alignment: Mount the sensor box so the X-axis (usually a printed arrow on the ICM-20948 board) points perfectly towards the bow. Slight deviations can be corrected in software (Heading Offset).

Orientation: Choose your mounting style in Menu 1 -> 5. Options: 0: Flat (Standard), 1: UpsideDown (Ceiling), 2: Bulkhead (Vertical wall, wires pointing down).

Positioning: Ideally install near the boat's Center of Gravity (CG), as low to the waterline as possible.

Lever Arm: If installed far from the CG (e.g., up a mast), input the physical X, Y, and Z distances (in meters) in the Configuration Menu so the software can mathematically negate centrifugal mast-sway forces.

5. Initial Setup via Bluetooth (BLE)

The device uses Bluetooth Low Energy (BLE).

Connecting to the Sensor

iOS (iPhone/iPad): Install "nRF Connect for Mobile", "LightBlue", or "Bluefruit Connect" from the App Store.

Android: Install "Serial Bluetooth Terminal" (select BLE mode) or the apps listed above.

Open the app, scan for "Precision9-BLE", and connect.

Subscribe to/Enable "Notifications" on the TX characteristic (UUID ending in 0003) to start receiving the configuration menu. Send commands via the RX characteristic (UUID ending in 0002).

6. Step-by-Step Calibration Guide

The V10.0 intelligence requires a solid baseline to enable autonomous learning. You only need to perform this procedure once.

Step 1: Boat Configuration

In the main menu, send 1 (Boat Config).

Dimensions: Set the length (meters) and tonnage. This dynamically scales the inertia filters.

Mounting Orientation: Select how the sensor is physically mounted (Flat, Ceiling, Bulkhead).

Environmental Location: Tag the BME280 data (Cabin, Outside, Engine Room).

Step 2: Bias Calibration (Zeroing Gyros)

In the main menu, send 3 (Calibrations), then 1 (Calibrate Bias).

Mandatory Condition: The boat must be completely still at the dock (engine off, no crew walking).

The sensor collects 400 samples over 5 seconds to calculate the physical imperfections of the gyroscope and accelerometer.

Step 3: Level Horizon

In menu 3, select 3 (Level Horizon).

Ensure the boat is evenly weighted. The system will record the current tilt as the absolute "Zero" for Pitch and Roll.

Step 4: Master Magnetic Calibration

In menu 3, select 2 (Calibrate Compass).

Take the boat to open, calm waters.

Once the command is sent, complete a full 360-degree circle at idle speed (taking about 45 to 60 seconds).

This maps the initial "Hard Iron" profile of your vessel (e.g., engine block interference).

Step 5: Enable "The Magic" (Auto-Learn) & Save

Go to option 4 in the Calibrations menu to ensure Continuous Auto-Calibration is ACTIVE.

Send 0 to return to the Main Menu.

Send 6 (SAVE TO NVS). The configuration is now permanently stored in the EEPROM.

(Optional) GPS Fallback Setup: In menu 4 (Filter Params), option 3, you can input your local Magnetic Variation manually (e.g., -2.5). If the N2K network GPS fails, the compass will seamlessly fall back to this manual value to keep broadcasting True Heading.

7. NMEA 2000 Plotter Configuration

After rebooting the module (Menu 7), connect it to the NMEA 2000 drop cable.

On your Simrad, B&G, Lowrance, or Raymarine MFD, go to Settings -> Network -> Data Sources.

Look for Compass/Heading, Attitude, and Heave.

Select the device named "Precision-9 Master" (or "EVO1 Compass", depending on your Network Mode choice).

For environment/temperature, select the corresponding source tagged by the BME280 setup.

Your MIL-SPEC Elite compass is now fully operational, broadcasting rock-solid data and ready to dynamically adapt to any extreme G-forces or magnetic changes encountered at sea!
