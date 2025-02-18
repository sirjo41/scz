package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "fingers")
public class pos extends LinearOpMode {

    public volatile static double figners_pos = 0;
    public volatile static double elbow_pos = 0;
    public volatile static double shoulder_pos = 0;

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

            telemetry.addData("Fingers pos", figners_pos);
            telemetry.addData("elbow pos",elbow_pos );
            telemetry.addData("shoulder pos", shoulder_pos);
            telemetry.update();
        }
    }
}
