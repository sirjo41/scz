package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "fingers")
public class pos extends LinearOpMode {

    public volatile static double figners_pos = 0.1;
    public static double ELBOW_INTAKE = 0.8;
    public static double SHOULDER_INTAKE = 1;
    public static double WRIST_INTAKE = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo fingers = hardwareMap.servo.get("fingers");
        Servo elbow = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");
        Servo wrist = hardwareMap.servo.get("wrist");
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            fingers.setPosition(figners_pos);
            elbow.setPosition(ELBOW_INTAKE);
            shoulder.setPosition(SHOULDER_INTAKE);
            wrist.setPosition(WRIST_INTAKE);

            telemetry.addData("slide1",slide1.getCurrentPosition());
            telemetry.addData("slide2",slide2.getCurrentPosition());
            telemetry.addData("arm",arm.getCurrentPosition());
            telemetry.addData("Fingers pos", figners_pos);
            telemetry.addData("elbow pos",ELBOW_INTAKE );
            telemetry.addData("shoulder pos", SHOULDER_INTAKE);
            telemetry.addData("wrist",WRIST_INTAKE);
            telemetry.update();
        }
    }
}
