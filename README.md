# Lakki ESP32 firmware

This firmware runs the Lakki cap and uses onboard IMU sensors for heading.

## Calibration and startup behavior

When the cap boots, there are two sensor preparation phases. The LEDs indicate what to do:

1. **IMU baseline initialization (~2 seconds, keep cap stationary)**
   - All direction LEDs are turned **ON steadily**.
   - Keep the cap still (do not move it).
   - Firmware averages accelerometer and gyro samples to:
     - initialize roll/pitch angles, and
     - compute gyro bias compensation.

2. **Magnetometer calibration mode switch (short stationary pause)**
   - Before movement calibration starts, all direction LEDs are turned **ON steadily** for a short delay.
   - Keep the cap still during this pause.

3. **Magnetometer movement calibration (move cap)**
   - LEDs start **blinking**.
   - Move the cap in figure-8 patterns and through full rotations so all axes are covered.
   - Continue moving while LEDs blink.

After these phases, heading updates run normally and direction LEDs indicate navigation direction.

## Practical user tips

- If calibration fails or heading feels unstable, reboot and repeat the process.
- During any **steady all-LED ON** phase: keep the cap stationary.
- During **blinking all-LED** phase: move the cap actively for calibration.

## Error indication

If calibration quality checks fail, firmware calls `indicate_fault_all_leds()`, which lights **all direction LEDs steadily for 5 seconds** and then turns them off.

What this means for the user:
- A required calibration step did not gather enough valid data, or computed scaling was invalid.
- Reboot and repeat calibration.
- During steady-ON phases keep the cap still; during blinking phase move the cap broadly in figure-8 and full rotations.
