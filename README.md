# Smart Air Conditioning Control System

This project is a smart embedded air conditioner controller designed for an Embedded Systems final project. It uses temperature, motion, and distance sensors to operate safely and automatically, while providing real-time user feedback via LCD, buzzer, and LEDs.

## Components Used

- DHT11 Temperature & Humidity Sensor
- HC-SR04 Ultrasonic Distance Sensor
- HC-SR501 PIR Motion Sensor
- 16x2 LCD Display (I2C)
- Buzzer
- 3 Push Buttons (MODE, UP, DOWN)
- DC Fan Motor + Motor Driver (L298N)
- Green LED (System Active)
- Red LED (System Off/Safety)
- Breadboard, jumper wires, and power supply

## Key Features

- Manual and Auto modes with temperature control (16°C–30°C)
- Motion sensor controls fan ON/OFF in Auto mode
- Ultrasonic sensor triggers safety shutdown when too close
- LCD displays system mode, temperature, and alerts
- Buzzer gives audio feedback (mode change, ON/OFF, adjust)
- Green/Red LEDs indicate active/inactive system states

## Usage

1. Upload `.ino` file to Arduino board.
2. Connect components as described in the code.
3. Press buttons to change mode or adjust temperature.
4. Observe behavior based on motion and proximity.

## Demo Summary

All components were mounted into a physical demo box for testing:
- Working fan control
- Motion-triggered automation
- Safety shutdown on proximity
- Real-time LCD and buzzer feedback

