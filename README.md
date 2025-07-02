<p align="right">
  <a href="https://www.linkedin.com/in/kazi-abrar-mahmud/" target="_blank">
    <img src="https://img.shields.io/badge/Connect%20on-LinkedIn-blue?style=flat&logo=linkedin" alt="LinkedIn">
  </a>
</p>

# 🤖 Day 5: PID-Based Line Follower | Robotics Bootcamp
## Tuned:
<div align="center">
  <img src="https://github.com/AbrarMahmud/RoboBootCamp_Day5/blob/master/misc_data/tuned.gif" alt="github-small" width="50%">
</div>

## Untuned:
<div align="center">
  <img src="https://github.com/AbrarMahmud/RoboBootCamp_Day5/blob/master/misc_data/untuned.gif" alt="github-small" width="50%">
</div>

Welcome to Day 5 of the Robotics Bootcamp! 🎯  
In this session, you will build a **PID-controlled Line Following Robot** using an **IR sensor array**, an **L298N motor driver**, and an **Arduino Uno**.

---

## 🛠️ Final Hardware Build
Visualize the final setup:
<div align="center">
  <img src="https://github.com/AbrarMahmud/RoboBootCamp_Day5/blob/master/misc_data/1751475046733.jpg" alt="github-small" width="50%">
</div>
<div align="center">
  <img src="https://github.com/AbrarMahmud/RoboBootCamp_Day5/blob/master/misc_data/1751475046712.jpg" alt="github-small" width="50%">
</div>

---

## 📌 Pin Assignment Reference

### IR Sensor Array → Arduino
| IR Sensor | Arduino Pin |
|-----------|-------------|
| Vcc       | 5V          |
| GND       | GND         |
| D1        | A0          |
| D2        | A1          |
| D3        | A3          |
| D4        | A4          |
| D6        | A5          |

### Motor Driver (L298N) → Arduino
| Motor Driver Pin | Arduino Pin |
|------------------|-------------|
| ENB              | 11          |
| IN3              | 12          |
| IN4              | 13          |
| ENA              | 10          |
| IN1              | 8           |
| IN2              | 9           |

---

## 🧠 What You’ll Learn
- Fundamentals of **PID Control** in robotics.
- How to **tune PID parameters** for stable line tracking.
- Working with analog IR sensor arrays.
- Real-time motor control using Arduino PWM.

---

## 🧪 Tuning PID
- **P (Proportional):** Controls how hard the robot turns.
- **I (Integral):** Eliminates steady-state error (rarely used for line followers).
- **D (Derivative):** Helps reduce overshoot and smoothen turns.

Start with only **P**, then incrementally introduce **D**. Use **serial print** for real-time debugging.

---

## 🚀 Get Started
1. Upload `line_follower_pid.ino` to your Arduino.
2. Power the bot and place it on a black line over white surface.
3. Watch the bot follow the path and fine-tune the PID parameters in the code as needed.

---

## 🧠 Tip for Tuning
> **Use trial and error.** Small changes in `Kp`, `Kd` make big differences. Print sensor readings to serial monitor to visualize error.

---

Happy building and tuning! 🎉  
Let your robot follow the line like a pro!
