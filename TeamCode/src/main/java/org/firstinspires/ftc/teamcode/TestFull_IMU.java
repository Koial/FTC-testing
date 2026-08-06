package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.function.ToIntFunction;

@TeleOp
public class TestFull_IMU extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Limelight3A limelight;
    
    double forward, strafe, rotate;
    boolean autoTrack = false;
    boolean lastY = false;
    
    // Tracking PID Constants
    double kP_tracking = 0.03;
    double kI_tracking = 0.005;
    double kD_tracking = 0.001;
    double maxTrackingPower = 0.3; // Limit maximum speed to 30%
    
    // Tracking PID State
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();

    public double step = 0;
    public static final double TICKS_PER_REV = 8192.0;
    public static final double GEAR_RATIO = 3.59;
    double ANGLE_30_DEGREES = 0.09504, ANGLE_25_DEGREES = 0.0792, ANGLE_20_DEGREES = 0.06336;
    double ANGLE_5_DEGREES = 0.01548, ANGLE_2_5_DEGREES = 0.00792;
    private DcMotorEx encoder;
    DcMotorEx leftTurretMotor, rightTurretMotor;

    boolean shooting = false;
    enum BurstState { IDLE, BALL_1_AND_2, PAUSE, BALL_3 }
    BurstState burstState = BurstState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();

    double BURST_TIME_MS = 600;
    double PAUSE_TIME_MS = 250;
    double BALL_3_TIME_MS = 300;

    // PIDF
    double F = 13.6; // 12
    double P = 49; // 42.85
    public double highVelocity = 2050;
    public double lowVelocity = 0;
    double curTargetVelocity = highVelocity;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftTurretMotor = hardwareMap.get(DcMotorEx.class, "leftTurret");
        rightTurretMotor = hardwareMap.get(DcMotorEx.class, "rightTurret");

        leftTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTurretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightTurretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftTurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightTurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        drive.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        pidTimer.reset();
        FtcDashboard.getInstance().startCameraStream(limelight, 0);
    }

    @Override
    public void loop() {
        // Limelight Toggle
        if (gamepad1.y && !lastY) {
            autoTrack = !autoTrack;
        }
        lastY = gamepad1.y;

        drive.setServoPos(step);

        // Encoder
        int currentTicks = encoder.getCurrentPosition();
        double encoderRotations = currentTicks / TICKS_PER_REV;
        double turretRotations = encoderRotations / GEAR_RATIO;
        double degrees = turretRotations * 360;

        // Drivetrain
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

        // Intake
        if(gamepad1.right_trigger_pressed) {
            drive.setMotorSpeed(1.0);
        } else if(gamepad1.left_trigger_pressed) {
            drive.setMotorSpeed(-1.0);
        } else {
            drive.setMotorSpeed(0);
        }

        // Turret Tracking Logic
        if (autoTrack) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // 'error' is how many degrees the AprilTag is from the center (tx)
                double error = result.getTx();
                
                // Calculate time elapsed since last loop to keep math consistent
                double dt = pidTimer.seconds();
                pidTimer.reset();

                // --- PROPORTIONAL (P) ---
                // Immediate response to current error: (error * kP)

                // --- INTEGRAL (I) ---
                // Accumulates error over time to overcome friction/steady-state error
                integralSum += error * dt;
                
                // Windup protection: Reset integral when centered or error is tiny
                if (Math.abs(error) < 0.5) {
                    integralSum = 0; 
                }

                // --- DERIVATIVE (D) ---
                // Responds to the CHANGE in error. Acts as a damper to stop jiggling/overshoot.
                double derivative = (error - lastError) / dt;
                lastError = error;

                // Combine all three terms for the final motor power
                double trackingPower = (error * kP_tracking) + (integralSum * kI_tracking) + (derivative * kD_tracking);
                
                // --- SPEED LIMIT ---
                // Limit the maximum power to prevent aggressive movement/jitter
                trackingPower = Math.max(-maxTrackingPower, Math.min(maxTrackingPower, trackingPower));

                // --- DEADZONE ---
                // If the error is less than 0.5 degrees, stop the servos entirely.
                // This prevents high-frequency buzzing/jitter when perfectly centered.
                if (Math.abs(error) < 0.5) {
                    trackingPower = 0;
                }

                drive.setServoRot(trackingPower);
                telemetry.addData("Turret Mode", "AUTO (Tracking)");
                telemetry.addData("TX Error", error);
            } else {
                // If no target is seen, stop and reset PID state
                drive.setServoRot(0);
                integralSum = 0;
                lastError = 0;
                pidTimer.reset();
                telemetry.addData("Turret Mode", "AUTO (Searching...)");
            }
        } else {
            // Reset variables when switching to manual mode
            integralSum = 0;
            lastError = 0;
            if(gamepad1.right_bumper) {
                drive.setServoRot(1);
            }else if(gamepad1.left_bumper) {
                drive.setServoRot(-1);
            }else {
                drive.setServoRot(0);
            }
            telemetry.addData("Turret Mode", "MANUAL");
        }

        if(gamepad1.dpadUpWasPressed()) {
            step += ANGLE_2_5_DEGREES;
        }
        if(gamepad1.dpadDownWasPressed()) {
            step -= ANGLE_2_5_DEGREES;
        }
        if(gamepad1.dpadLeftWasPressed()) {
            step = 0;
        }
        if(gamepad1.dpadRightWasPressed()) {
            step = ANGLE_30_DEGREES;
        }

        telemetry.addData("Heading", drive.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Degrees", "%.1f", degrees);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftTurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightTurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        if (gamepad1.a) {
            curTargetVelocity = highVelocity;
            leftTurretMotor.setVelocity(curTargetVelocity);
            rightTurretMotor.setVelocity(curTargetVelocity);
        } else {
            curTargetVelocity = lowVelocity;
            leftTurretMotor.setVelocity(curTargetVelocity);
            rightTurretMotor.setVelocity(curTargetVelocity);
        }

        double curVelocity = leftTurretMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        if(curVelocity >= 1850) {
            step = ANGLE_30_DEGREES;
        }
        if(curVelocity < 1850 && curVelocity >= 1650) {
            step = ANGLE_25_DEGREES;
        }
        if(curVelocity < 1650) {
            step = 0;
        }

        // Shooting

        switch (burstState) {
            case IDLE:
                if(gamepad1.b) {
                    shootTimer.reset();
                    burstState = BurstState.BALL_1_AND_2;
                } else {
                    drive.setMotorSpeed(0.0);
                }
                break;

            case BALL_1_AND_2:
                drive.setMotorSpeed(1.0);
                if(shootTimer.milliseconds() >= BURST_TIME_MS) {
                    drive.setMotorSpeed(0.0);
                    shootTimer.reset();
                    burstState = BurstState.PAUSE;
                }
                break;

            case PAUSE:
                drive.setMotorSpeed(0.0);
                if(shootTimer.milliseconds() >= PAUSE_TIME_MS) {
                    shootTimer.reset();
                    burstState = BurstState.BALL_3;
                }
                break;

            case BALL_3:
                drive.setMotorSpeed(1.0);
                if(shootTimer.milliseconds() >= BALL_3_TIME_MS) {
                    drive.setMotorSpeed(0.0);
                    burstState = BurstState.IDLE;
                }
                break;
        }

        // Telemetry

        double angleInDegrees = 315.6566;
        double angleInRadians = Math.toRadians(angleInDegrees);
        double resultInRadians = step * angleInRadians;
        double resultInDegrees = Math.toDegrees(resultInRadians);

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Degress Hood", resultInDegrees+"°");
    }
}
