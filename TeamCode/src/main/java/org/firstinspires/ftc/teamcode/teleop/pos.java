package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "fingers")
public class pos extends LinearOpMode {

    public static double WRIST_INTAKE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.servo.get("wrist");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            wrist.setPosition(WRIST_INTAKE);
            telemetry.addData("wrist",WRIST_INTAKE);
            telemetry.addData("arm",arm.getCurrentPosition());
            telemetry.addData("slide1",slide1.getCurrentPosition());
            telemetry.addData("slide2",slide2.getCurrentPosition());
            telemetry.update();
        }
    }
}
