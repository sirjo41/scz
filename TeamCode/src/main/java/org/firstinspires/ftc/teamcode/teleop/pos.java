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

        telemetry.addData("Fingers pos", fingers.getPosition());
    }
}
