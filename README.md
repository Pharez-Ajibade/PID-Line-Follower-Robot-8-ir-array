# PID-Line-Follower-Robot-8-ir-array
Overview

# Youtube Video Link; https://youtu.be/oNtx3up80jQ?si=pXBH429U2dCFqUBu
![20251210_064209](https://github.com/user-attachments/assets/0efb0eb2-0df7-4ab7-bfda-48aa8e7835cb)
![20251210_064204](https://github.com/user-attachments/assets/d18de657-d7df-4bae-82c9-fd888f6e2519)
![20251210_064109](https://github.com/user-attachments/assets/c669d87b-2e1f-47e0-849c-ce43dbdcd320)

Phase 1 of the high-performance Speed Demon Line Follower Robot focuses on exploring advanced line-following techniques using PID control. This platform allows testing of motor control, sensor calibration, and line-following algorithms, with future iterations targeting high-speed navigation.

This robot relies on infrared (IR) sensors rather than cameras, making it a low-cost, precise, and responsive solution for line-following tasks.

# Features

8-IR Sensor Array for accurate line detection

PID Control Loop for smooth motor adjustments

Weighted Average Line Position calculation

Calibration System with LED indicators

ESP32 Microcontroller for high-speed PWM motor control

Modular & Expandable Hardware Platform

# Bill of Engineering Measurement and Instrumentation
S/N	Component	Quantity
1	Smart car robot chassis kit 4WD	1
2	ESP32 DEV MODULE	1
3	18650 1S Li-po battery holder	3
4	8-IR sensor array	1
5	18650 Li-po Batteries	3
6	TB6612FNG motor driver board	1
7	SPST switch	1
8	Push Button switch	1
9	LM2596 buck converter	1
10	T-connector	1
11	M3 screws set	10
12	3D printed slot	1
13	2WD mini vacuum steel ball caster wheel	1
14	Electrical tape	1
15	Jumper wires, Vero board, soldering lead	1 set
16	White banner with lines (6ft x 6ft)	1

# Hardware Setup

Chassis: 4WD smart car kit with acrylic base

Motors: Two DC geared motors controlled via TB6612FNG

IR Sensor Array: 8 TCRT5000 sensors mounted 3–5mm above track

Microcontroller: ESP32 for sensor reading and PWM motor control

Power: 2S/3S LiPo battery packs with protection

Indicators: Red, yellow, and green LEDs for calibration and status

Switches: SPST for power; push button for logic/calibration

Software & Control Logic

PID-based line following:

Read Sensors: Collect values from 8 IR sensors

Weighted Average: Compute line position (-40 to 40)

# PID Calculation:

P (Proportional): Reacts to current error

I (Integral): Corrects accumulated error over time

D (Derivative): Dampens oscillations

Motor Adjustment: PWM values updated for left/right motors

Calibration: Measures black/white surfaces to set sensor min/max

Update rate: 50ms per PID loop (20Hz)

# CIRCUIT SCHEMATIC
![Circit Schematic breadboard view](https://github.com/user-attachments/assets/85c81415-2916-4880-82ac-6db184d014f9)

# 8 IR ARRAY HOLDER 
<img width="1519" height="1022" alt="Screenshot 2026-01-02 115138" src="https://github.com/user-attachments/assets/bcfd305f-6827-4f73-96d2-57aa9c739ea0" />

