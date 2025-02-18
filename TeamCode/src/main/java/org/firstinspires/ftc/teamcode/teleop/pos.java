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
        Servo elbow = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");

        waitForStart();

        double po = 0;
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            fingers.setPosition(po);
            elbow.setPosition(po);
            shoulder.setPosition(po);

            if(gamepad1.a){
                po += 0.1;
            }
            else if(gamepad1.b){
                po -= 0.1;
            }
            telemetry.addData("Fingers pos", fingers.getPosition());
            telemetry.addData("elbow pos", elbow.getPosition());
            telemetry.addData("shoulder pos", shoulder.getPosition());
            telemetry.update();
        }
    }
}
