package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "slide", group = "TeleOp")
public class Tele extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize motor
        DcMotor slide2 = hardwareMap.get(DcMotor.class, "slide2");
        DcMotor slide1 = hardwareMap.get(DcMotor.class, "slide1");
        // Set the motor direction
        slide2.setDirection(DcMotor.Direction.REVERSE); // Change to FORWARD for "up"
        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check gamepad input for stage control
            if (gamepad1.a) {
                slide2.setTargetPosition(1600); // Positive value for upward movement
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setPower(1);
                slide1.setPower(0.7);



                while (opModeIsActive() && slide2.isBusy()) {
                    telemetry.addData("slide", slide2.getCurrentPosition());
                    telemetry.update();
                }

                // Stop the motor after reaching the target
                slide1.setPower(0);
                slide2.setPower(0);
            }
        }
    }
}
