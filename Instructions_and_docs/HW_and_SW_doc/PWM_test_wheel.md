# PWM Steering Measurements

This document contains PWM measurements for steering control using both the original RC controller and the Teensy flight controller setup.

---

# Original RC Controller (Joystick)

**Period:** 17.5 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.6 ms | 9.14% |
| Full Right | 1.0 ms | 5.71% |
| Full Left | 2.1 ms | 12.00% |

---

# Teensy + Wheel (No Steering Trim)

**Period:** 19.98 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.47 ms | 7.3% |
| Full Right | 0.891 ms | 4.46% |
| Full Left | 1.89 ms | 9.46% |

---

# Teensy + Wheel (Steering Trim = 20)

**Period:** 19.98 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.70 ms | 8.50% |
| Full Right | 1.10 ms | 5.50% |
| Full Left | 1.88 ms | 9.41% |

---

# Teensy + Wheel (Steering Trim = 40)

**Period:** 19.96 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.84 ms | 9.22% |
| Full Right | 1.21 ms | 5.61% |
| Full Left | 1.88 ms | 9.41% |

---

# Teensy + Wheel (Trim 40, Left Range 135 + 120)

**Period:** 20.00 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.88 ms | 9.4% |
| Full Right | 1.23 ms | 6.15% |
| Full Left | 1.88 ms | 9.4% |

---

# Teensy + Wheel (Trim 0, Left Range 135 + 65)

**Period:** 20.00 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.48 ms | 7.4% |
| Full Right | 0.95 ms | 4.75% |
| Full Left | 1.88 ms | 9.4% |

---

# Teensy + Wheel (Trim 35, Left Range 135 + 65)

**Period:** 20.00 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.72 ms | 8.6% |
| Full Right | 1.19 ms | 6.0% |
| Full Left | 1.88 ms | 9.4% |

---

# Teensy + Wheel (Left Range 300, Right Range 0)

**Period:** 20.00 ms

| State | High Time | Duty Cycle |
|---|---|---|
| Neutral | 1.68 ms | 8.4% |
| Full Right | 0.63 ms | 3.15% |
| Full Left | 1.88 ms | 9.4% |

---

# Teensy + Wheel (Left Range 45, Right Range -180)

**Period:** 20.00 ms

| State | High Time |
|---|---|
| Neutral | 0.64 ms |
| Full Right | 0.64 ms |
| Full Left | 0.95 ms |

---

# Teensy + Wheel (Left Range 45, Right Range 0)

**Period:** 20.00 ms

| State | High Time |
|---|---|
| Neutral | 0.79 ms |
| Full Right | 0.63 ms |
| Full Left | 0.95 ms |

---

# Teensy + Wheel (Left Range 180, Right Range 90)

**Period:** 20.00 ms

| State | High Time |
|---|---|
| Neutral | 1.57 ms |
| Full Right | 1.25 ms |
| Full Left | 1.88 ms |

---

# Teensy + Wheel (Left Range 180, Right Range 120)

**Period:** 20.00 ms

| State | High Time |
|---|---|
| Neutral | 1.68 ms |
| Full Right | 1.46 ms |
| Full Left | 1.88 ms |

---

# Teensy + Wheel (Left Range 180, Right Range 0)

Configuration:
