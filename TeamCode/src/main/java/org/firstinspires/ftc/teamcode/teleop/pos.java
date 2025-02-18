package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "fingers")
public class pos extends LinearOpMode {

    public volatile double figners_pos = 0;
    public volatile double elbow_pos = 0;
    public volatile double shoulder_pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo fingers = hardwareMap.servo.get("fingers");
        Servo elbow = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");

        waitForStart();

        double po = 0;
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            fingers.setPosition(figners_pos);
            elbow.setPosition(elbow_pos);
            shoulder.setPosition(shoulder_pos);

            telemetry.addData("Fingers pos", fingers.getPosition());
            telemetry.addData("elbow pos", elbow.getPosition());
            telemetry.addData("shoulder pos", shoulder.getPosition());
            telemetry.update();
        }
    }
}
