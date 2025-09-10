# Note on the Color Sensor – WRO Future Engineers 2025

## Why the Color Sensor Is Missing in the Schematic
The wiring schematic for our vehicle was created using **Fritzing**.  
However, Fritzing does not currently provide a built-in component for the **TCS3200 Color Sensor** (or any similar color sensor).  
Because of this limitation, we were unable to include the color sensor in the uploaded schematic, even though it is an essential part of our actual hardware design.

---

## Purpose of the Color Sensor in Our Vehicle
Although it is missing from the schematic, the **color sensor is fully integrated** in our physical robot and serves a crucial function:

### **Primary Function: Lap Counting**
The main purpose of the color sensor is to **count how many laps the robot has completed** on the track.  
This is critical because:

1. **Race Strategy Requirement**  
   - According to the competition rules, the robot must **complete exactly 3 laps**  
   - After the 3rd lap, the robot must **automatically park** in the designated area

2. **How It Works**  
   - The sensor detects **specific color markers** placed on the track surface  
   - Each time the robot passes over 4 markers, it counts one lap  
   - After detecting the **third marker**, the robot activates its parking system

3. **Precision Timing**  
   - Accurate lap counting ensures the robot stops at the **exact required position**  
   - Without this sensor, the robot would not know when to begin parking procedures

---

## Conclusion
Even though the color sensor is not visible in the schematic due to software limitations, it is a **critical component** for lap counting in our robot.  
It provides the essential capability to:  
- Detect lap completion markers on the track  
- Count exactly 3 laps as required by competition rules  
- Trigger the parking system at the precise moment  

Thus, the **color sensor plays a vital role** in ensuring that our vehicle performs the complete race sequence correctly and stops after exactly 3 laps in the **WRO Future Engineers 2025** competition.
