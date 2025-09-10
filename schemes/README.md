# Note on the Color Sensor – WRO Future Engineers 2025

## Why the Color Sensor Is Missing in the Schematic
The wiring schematic for our vehicle was created using **Fritzing**.  
However, Fritzing does not currently provide a built-in component for the **TCS3200 Color Sensor** (or any similar color sensor).  
Because of this limitation, we were unable to include the color sensor in the uploaded schematic, even though it is an essential part of our actual hardware design.

---

## Purpose of the Color Sensor in Our Vehicle
Although it is missing from the schematic, the **color sensor is fully integrated** in our physical robot and serves several important functions:

1. **Track Detection**  
   - The sensor helps the robot distinguish between **different colors on the field surface**.  
   - For example, it can differentiate black lines from white backgrounds to support **line-following**.

2. **Obstacle Identification**  
   - Colored cubes placed on the field (e.g., red or green) are recognized by the sensor.  
   - Based on the detected color, the robot makes a decision:
     - **Red cube → pass on the right**  
     - **Green cube → pass on the left**  

3. **Decision Making and Strategy**  
   - The color sensor acts as a decision trigger during navigation.  
   - By interpreting color signals, the robot can adapt its path dynamically and execute **rule-based strategies** in real time.

---

## Conclusion
Even though the color sensor is not visible in the schematic due to software limitations, it is a **critical sensing component** in our robot.  
It provides the ability to:  
- Follow lines and detect markers,  
- Recognize obstacles and field objects,  
- Make autonomous decisions during navigation.  

Thus, the **color sensor plays a central role** in ensuring that our vehicle performs reliably in the **WRO Future Engineers 2025** competition.