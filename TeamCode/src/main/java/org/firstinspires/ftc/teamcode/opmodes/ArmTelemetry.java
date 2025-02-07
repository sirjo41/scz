package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmTelemetry", group = "TeleOp")
public class ArmTelemetry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        // Reset encoder and set mode
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");
        Servo wrist = hardwareMap.servo.get("Wrist");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("slide1" , slide1.getCurrentPosition());
            telemetry.addData("slide2",slide2.getCurrentPosition());
            telemetry.addData("wrist",wrist.getPosition());

            telemetry.update();
        }
    }
}