## Googly Eyes (Arduino)

The **Googly Eyes** project transforms your pocket lint into a pair of animated eyes that react realistically* to gravity and gentle movement. The pupils seem to “follow” the gravitational vector, and they even simulate a bit of inertia when you tilt the device.
 
 *\*realism subject to interpretation*

### How It Works

1. **Calibration Sequence:**  
   Since the device is roughly cube-like with a screen on the front and a button on the right, only four faces are available for calibration:  
   - **BOTTOM** (the face normally on the table)  
   - **LEFT**  
   - **TOP** (upside down relative to bottom)  
   - **BACK**

   When the device boots up (or when you trigger recalibration), it displays clear instructions on the screen for placing the cube on each of these faces. After you press the button, a three‑second countdown lets the device settle, and then 50 accelerometer samples are collected and averaged. These averaged values are stored and later used to compute sensitivity factors.

2. **Mapping Sensor Data to Pupil Movement:**  
   - The **vertical mapping** uses the X‑axis accelerometer readings. Calibration data shows that when the device is on its BOTTOM face the reading is higher (e.g. +0.12) and on its TOP face it is lower (e.g. –0.13). We remap that range so that a BOTTOM reading positions the pupil at the bottom of the eye (an offset of +25 pixels) and a TOP reading at the top (–25 pixels).  
   - The **horizontal mapping** uses the Y‑axis readings. The difference between the LEFT and BOTTOM calibrations is used to scale the lateral movement so that tilting the device causes the pupil to shift left or right.

3. **Display & Smoothing:**  
   The pupils are rendered as circles within larger circular “eyes”. Clipping and the removal of scroll bars ensure the pupils never leave the bounds of the eye. Additionally, a smoothing function is applied to simulate inertia for a more natural, gradual movement.

4. **Debug Mode:**  
   After calibration, the device enters a debug mode where the calibration values (including computed sensitivity factors) are displayed. This screen stays visible until you press the button, giving you ample time to review the data. (You can then exit debug mode to see the animated eyes.)

### Tweaking & Adjustments

Based on your hardware and sensor behavior, you might need to tweak:
- **Countdown Duration & Sample Count:** Adjust these to give the sensor time to settle or to average out noise better.
- **Sensitivity Factors & Mapping:** The provided code maps the bottom reading to an offset of +25 (pupil at the bottom) and the top reading to –25 (pupil at the top). If your sensor readings differ, you might need to adjust the scaling factors.
- **Smoothing:** If the eyes seem too jumpy or too sluggish, tweak the smoothing constant.

### The Math Behind the Googly Eyes

To make the pupils follow “gravity,” we convert the accelerometer’s 3‑axis readings into a 2D pupil offset. Although the IMU provides 6‑axis data (3‑axis accelerometer plus 3‑axis gyroscope), we use only the accelerometer data to approximate the gravitational vector. The process is as follows:

1. **Calibration:**

   The device is calibrated by placing it on four faces:
   
   - **BOTTOM:** Device resting on the table.
   - **LEFT:** Left face down.
   - **TOP:** Upside down relative to bottom.
   - **BACK:** Back face down.
   
   For each orientation the accelerometer outputs a vector. For example, if we denote the calibration readings as:
   
   - **BOTTOM:** $B = (B_x,\,B_y,\,B_z)$
   - **LEFT:** $L = (L_x,\,L_y,\,L_z)$
   - **TOP:** $T = (T_x,\,T_y,\,T_z)$
   - **BACK:** $K = (K_x,\,K_y,\,K_z)$
   
   We use the **BOTTOM** orientation as the neutral reference.

2. **Mapping to Pupil Position:**

   We perform two separate linear mappings—one for vertical movement (using the X‑axis readings) and one for horizontal movement (using the Y‑axis readings).

   **Vertical Mapping (Pupil moves up and down):**
   
   - **Range:** From the BOTTOM to the TOP calibration along the X‑axis.  
     Compute the full range as:  
     $$\Delta_{vertical} = B_x - T_x$$
     (For example, if $B_x \approx +0.12$ and $T_x \approx -0.13$, then $\Delta_{vertical} \approx 0.25$.)
   
   - **Mapping:** We want to map this sensor range linearly to a pixel offset range from +25 (pupil at the bottom of the eye) to –25 (pupil at the top).  
     Let the current accelerometer reading be $a_x$. Then the difference from the neutral is:  
     $\Delta_x = a_x - B_x$
     And the vertical offset is computed as:
     $$\text{offset}_y = \Delta_x \times \left(\frac{50}{\Delta_{vertical}}\right) + 25$$
     This ensures that when $a_x = B_x$, the pupil is at +25 (the bottom), and when $a_x = T_x$, the pupil is at –25 (the top).
   
   **Horizontal Mapping (Pupil moves left and right):**
   
   - **Range:** Based on the difference in the Y‑axis between the LEFT and BOTTOM calibrations.  
     Compute:
     $\Delta_{horizontal} = L_y - B_y$
   
   - **Mapping:** Let the current Y‑axis reading be $a_y$ and the difference be:
     $\Delta_y = a_y - B_y$
     Then, the horizontal offset is computed as:
     $$\text{offset}_x = -\Delta_y \times \left(\frac{25}{|\Delta_{horizontal}|}\right)$$
     The negative sign ensures that tilting the device toward the LEFT (i.e. increasing $a_y$) moves the pupil left.

3. **Constraining & Smoothing:**

   To keep the pupil inside the circular “eye” (of radius 25 pixels), we constrain the resulting offset vector $(\text{offset}_x,\, \text{offset}_y)$ such that:
   $$\sqrt{\text{offset}_x^2 + \text{offset}_y^2} \leq 25$$
   
   Additionally, a smoothing factor (an exponential moving average) is applied to the computed offsets to simulate inertia and ensure that the pupil movement appears natural rather than jumpy.

---

### Summary

- **Calibration** uses four known orientations to establish reference accelerometer readings.
- **Vertical mapping** leverages the difference in the X‑axis readings (between BOTTOM and TOP) to assign pupil positions from bottom (+25 pixels) to top (–25 pixels).
- **Horizontal mapping** uses the Y‑axis difference (between LEFT and BOTTOM) to determine left/right movement.
- **Constraints and smoothing** keep the pupil within a circular region and create natural motion.

This linear mapping method is a simple yet effective approximation to give the illusion that the pupils are “following” gravity based solely on the 3‑axis accelerometer data.
