package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "fingers")
public class pos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo fingers = hardwareMap.servo.get("fingers");
        Servo elbow  = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");
        telemetry.addData("Fingers pos", fingers.getPosition());
        telemetry.addData("elbow pos",elbow.getPosition());
        telemetry.addData("shoulder pos", shoulder.getPosition());
        telemetry.update();
    }
}
