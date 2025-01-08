package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "slide", group = "TeleOp")
public class Slides extends LinearOpMode {

    // Define constants for stage positions
    private static final int STAGE_1 = 900;
    private static final int STAGE_2 = 1800;
    private static final int STAGE_3 = 2700;

    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor slide2 = hardwareMap.get(DcMotor.class, "slide2");
        DcMotor slide1 = hardwareMap.get(DcMotor.class, "slide1");

        // Set the motor direction
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check gamepad input for stage control
            if (gamepad1.a) {
                moveToPosition(slide1, slide2, STAGE_1); // Move to stage 1
            } else if (gamepad1.b) {
                moveToPosition(slide1, slide2, STAGE_2); // Move to stage 2
            } else if (gamepad1.y) {
                moveToPosition(slide1, slide2, STAGE_3); // Move to stage 3
            }
        }
    }
    private void moveToPosition(DcMotor slide1, DcMotor slide2, int targetPosition) {
        int currentPosition = slide2.getCurrentPosition();
        int direction = (targetPosition > currentPosition) ? 1 : -1; // Determine the direction

        slide2.setTargetPosition(targetPosition);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setPower(1); // Adjust power based on direction
        slide1.setPower(direction * 0.7);

        while (opModeIsActive() && slide2.isBusy()) {
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", slide2.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors after reaching the target
        slide1.setPower(0);
        slide2.setPower(0);
        telemetry.addData("Reached", "Pos " + slide2.getCurrentPosition());
        telemetry.update();
    }


}
