# CTRL + ALT + DEFEAT 

This is the Arduino code powering our sumobot for competition.  
We’re using an all-wheel-drive setup with directional sensing and basic edge detection to stay in the ring and fight opponents.

## Features

- **All-Wheel Drive (4 independent DC motors)**  
  Controlled via PWM through a dual-channel H-Bridge driver. Each wheel is managed separately for tighter turning and better control.

- **3 Ultrasonic Sensors @ 45° Angles**  
  Detects opponent direction. Bot prioritises:
  - Center sensor → charge forward.
  - Left sensor → spin/turn left.
  - Right sensor → spin/turn right.

- **IR Edge Sensor**  
  If it detects the white ring boundary, the bot quickly moves to the right to avoid moving out of the ring. 

## Hardware Used

- Arduino Mega 2560  
- 4 × JGA25-370 DC motors with encoders (encoder not yet used in code)  
- 10A Dual Channel H-Bridge Motor Driver (PWM speed control)  
- 3 × Ultrasonic sensors  
- 1 × IR sensor  
- Batteriess

## How It Works

- Code logic can be found in `sumbot.ino`
- Pin assignments and motor setup inits can be found inside the code as well. 
- Opponent detection logic prioritises the *closest* sensor reading. 
- Movement logic is mapped clearly: `moveForward()`, `moveLeft()`, `moveRight()`, `stopAll()`

---

Team Members: Adrian, Dakota, Manan, Naveen, Senuthi, Sidhida 