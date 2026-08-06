# Implementation Plan - Stabilize Turret Tracking (Filtering & Tuning)

The goal is to eliminate the "jitter" in the turret tracking. Jitter is often caused by sensor noise (the Limelight values jumping slightly) or the controller reacting too aggressively to high-frequency changes.

## User Review Required

> [!TIP]
> I am adding a **Low-Pass Filter (LPF)**. This will "smooth out" the data from the Limelight before the PID controller sees it. It's like a shock absorber for your data.

## Proposed Changes

### `TeamCode` module

#### [MODIFY] [TestFull_IMU.java](file:///home/soare/StudioProjects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TestFull_IMU.java)
- **Add Low-Pass Filter State**:
    - `double lowPassError = 0;`
    - `double a = 0.5;` (The smoothing factor. 1.0 = no filter, 0.1 = very smooth but slow).
- **Refine PID Constants**:
    - Slightly lower `kP_tracking` to reduce aggressive snaps.
    - Slightly increase `kD_tracking` to add more "braking" force.
- **Update Logic**:
    - Apply the LPF to the `tx` value before calculating PID.
    - Limit the `integralSum` (clamping) to prevent it from growing too large if the turret is stuck.
    - Limit the final `trackingPower` to a maximum (e.g., 0.5) to keep the movement speed controlled.

## Verification Plan

### Manual Verification
1. Run `TestFull_IMU`.
2. Observe the turret movement. It should feel "softer" and less twitchy.
3. If it's too "mushy" or slow to respond, I will increase the `a` factor (e.g., to 0.7).
4. If it still jitters, I will decrease the `a` factor (e.g., to 0.3) or lower `kP`.
