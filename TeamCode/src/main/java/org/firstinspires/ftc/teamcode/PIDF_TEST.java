package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PIDF_TEST extends OpMode {
    DcMotorEx leftMotor, rightMotor, intakeMotor;
    Servo servoHood;

    public double highVelocity = 2000;
    public double lowVelocity = 1400;
    double curTargetVelocity = highVelocity;
    double F = 12; // 12
    double P = 0; // 42.85
    double I = 0;
    double D = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftTurret");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightTurret");
        servoHood = hardwareMap.get(Servo.class, "servo_hood");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motor_intake_dreapta");
    }

    @Override
    public void loop() {
        servoHood.setPosition(0.095);

        if(gamepad1.yWasPressed()) {
            if(curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        if(gamepad1.xWasPressed()) {
            curTargetVelocity = 0;
        }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if(gamepad1.right_trigger_pressed) {
            intakeMotor.setPower(1.0);
        }
        if(gamepad1.left_trigger_pressed) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        leftMotor.setVelocity(curTargetVelocity);
        rightMotor.setVelocity(curTargetVelocity);

        double curVelocity = leftMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------------------");
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("I", "%.4f", I);
        telemetry.addData("D", "%.4f", D);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f B", stepSizes[stepIndex]);
    }
}
