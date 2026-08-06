package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Limelight Turret Tracking", group = "Mechanisms")
public class Limelight extends OpMode {

    public Limelight3A limelight;
    private CRServo servoTurretLeft;
    private CRServo servoTurretRight;

    // P-controller gain. Adjust this to change tracking speed/stability.
    public static double kP = 0.02; 
    
    private double x;
    private double y;
    private double area;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servoTurretLeft = hardwareMap.get(CRServo.class, "servo_tureta_stanga");
        servoTurretRight = hardwareMap.get(CRServo.class, "servo_tureta_dreapta");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            x = result.getTx();
            y = result.getTy();
            area = result.getTa();

            telemetry.addData("Target", "Detected");
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Area", area);

            // Calculate power based on horizontal error (tx)
            // tx is usually between -30 and 30 degrees.
            double power = x * kP;

            // Apply power to servos. 
            // They spin together because they are geared.
            // One might need to be reversed if they are mounted mirrored.
            // Using opposite signs as per previous implementation attempt.
            servoTurretLeft.setPower(power);
            servoTurretRight.setPower(power);
        } else {
            telemetry.addData("Target", "Not Detected");
            
            // Stop the turret if no target is found
            servoTurretLeft.setPower(0);
            servoTurretRight.setPower(0);
        }
        
        telemetry.update();
    }
}
