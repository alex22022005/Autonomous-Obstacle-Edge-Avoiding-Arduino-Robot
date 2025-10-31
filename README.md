# Autonomous Obstacle & Edge Avoiding Arduino Robot

This is an autonomous 4WD Arduino robot that uses an ultrasonic sensor to avoid obstacles and a downward-facing IR sensor to detect edges, preventing it from falling off surfaces.



---

## 🤖 Core Features

* **Autonomous Navigation:** The robot moves forward on its own.
* **Obstacle Avoidance:** Uses an HC-SR04 ultrasonic sensor mounted on a servo to scan its surroundings. It checks for obstacles, compares the distance on its left and right, and turns towards the direction with more free space.
* **Edge Detection:** A downward-facing IR sensor detects "edges" (like the end of a table). When an edge is detected, the robot stops, reverses, and turns to avoid falling.

---

## 🛠️ Hardware Requirements

* Arduino (Uno, Nano, or similar)
* Adafruit Motor Shield V1 (or a compatible L293D-based shield)
* 4WD Robot Chassis with 4 DC Motors
* HC-SR04 Ultrasonic Sensor
* SG90 Micro Servo
* IR Proximity Sensor (for edge detection)
* Power Supply (e.g., 9V battery pack or 7.4V LiPo)
* Jumper Wires

---

## 🔧 Software & Setup

### 1. Libraries
This project requires the following Arduino library:

* **`AFMotor.h`**: The Adafruit Motor Shield V1 library.
    * You can install this from the Arduino IDE: Go to **Tools** > **Manage Libraries...** and search for "Adafruit Motor Shield".

The `Servo.h` library is built into the Arduino IDE.

### 2. Pin Configuration

Make sure your sensors and motors are connected as defined in the code:

| Component | Pin |
| :--- | :--- |
| IR Sensor | `A0` |
| Ultrasonic Echo | `A1` |
| Ultrasonic Trig | `A2` |
| Servo Motor | `10` |
| Motor 1 | `M1` (on Shield) |
| Motor 2 | `M2` (on Shield) |
| Motor 3 | `M3` (on Shield) |
| Motor 4 | `M4` (on Shield) |

---

## ⚙️ How It Works

The robot's logic is split into two main functions in its `loop()`:

1.  **`Obstacle()`**:
    * It continuously checks the distance directly in front using the `ultrasonic()` sensor.
    * If an obstacle is detected within the `OBSTACLE_DISTANCE_THRESHOLD` (5 cm):
        1.  It stops.
        2.  It uses the servo to look left (`leftsee()`) and measures the distance.
        3.  It uses the servo to look right (`rightsee()`) and measures the distance.
        4.  It compares the left and right distances and turns towards the direction with more room.

2.  **`EdgeDetection()`**:
    * It continuously reads the value from the downward-facing `IR_SENSOR`.
    * If the `irValue` is greater than the `IR_THRESHOLD` (meaning it detects an edge / no surface):
        1.  It stops.
        2.  It reverses for 1 second.
        3.  It turns left for 0.5 seconds to move away from the edge.

If no obstacle or edge is detected, the robot continues to move `forward()`.

---

## 🚀 How to Use

1.  Clone this repository or download the `.ino` file.
2.  Assemble the hardware according to the **Pin Configuration** table.
3.  Open the `.ino` file in your Arduino IDE.
4.  Install the **Adafruit Motor Shield** library.
5.  Upload the sketch to your Arduino board.
6.  Place the robot on the floor or a large table and power it on.
