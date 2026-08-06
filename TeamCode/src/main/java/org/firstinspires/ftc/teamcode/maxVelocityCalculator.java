package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class maxVelocityCalculator extends OpMode {
    DcMotorEx leftMotor, rightMotor, intake;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "motor_intake_dreapta");

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftTurret");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightTurret");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftTurret");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightTurret");

        currentVelocity = leftMotor.getVelocity();

        if(currentVelocity > maxVelocity){
            maxVelocity = currentVelocity;
        }

        if(gamepad1.right_trigger_pressed) {
            intake.setPower(1.0);
        }
        if(gamepad1.left_trigger_pressed) {
            intake.setPower(0.0);
        }

        if(gamepad1.a) {
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        }
        if(gamepad1.b) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        telemetry.addData("Current velocity", currentVelocity);
        telemetry.addData("Maximum velocity", maxVelocity);
    }
}
