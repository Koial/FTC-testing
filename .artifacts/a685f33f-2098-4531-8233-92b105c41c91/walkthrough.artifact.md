# Walkthrough - PID Turret Tracking Upgrade

I have upgraded the turret tracking system in `TestFull_IMU.java` from a basic proportional (P) controller to a full PID controller with a deadzone. This change is designed to eliminate the "jiggling" and overshooting you were seeing.

## Changes

### [TestFull_IMU.java](file:///home/soare/StudioProjects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TestFull_IMU.java)

- **PID Constants**: Added `kP_tracking`, `kI_tracking`, and `kD_tracking`.
    - `kD` (Derivative) provides the "damping" that stops the turret from overshooting and jiggling.
    - `kI` (Integral) ensures the turret reaches the exact center even if there's friction.
- **State Management**: Added `integralSum`, `lastError`, and `pidTimer` to keep track of the controller state between loops.
- **Refined Logic**:
    - **Reduced Speed and Gains**:
    - Added `maxTrackingPower = 0.3` to limit the turret to 30% speed during auto-tracking.
    - Lowered PID gains (`kP=0.03`, `kI=0.005`, `kD=0.001`) for smoother, less aggressive movement.
    - **Integral Reset**: The integral sum resets when the target is centered or lost to prevent "windup".
    - **Time-Based Calculations**: Used `ElapsedTime` to ensure the PID math stays consistent regardless of the robot's loop speed.

## How to Tune
If it still isn't perfect, you can adjust these numbers at the top of the class:
- **If it still jiggles**: Increase `kD_tracking` (try `0.005`).
- **If it's too slow**: Increase `kP_tracking` (try `0.06`).
- **If it stops slightly off-center**: Increase `kI_tracking` (try `0.02`).

## Verification Results

### Automated Tests
- The project builds successfully with the new logic.

### Manual Verification Steps
1. Run `TestFull_IMU`.
2. Toggle auto-tracking with **Y**.
3. Move the AprilTag and observe the turret movement. It should be much smoother and "snap" into place without shaking.
