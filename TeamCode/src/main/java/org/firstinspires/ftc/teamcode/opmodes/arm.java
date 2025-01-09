package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "arm", group = "TeleOp")
public class arm extends LinearOpMode {

    // Define constants for stage positions
    private static final int STAGE_1 = 900;
    private static final int STAGE_2 = 1800;
    private static final int STAGE_3 = 2700;

    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        // Set the motor direction
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check gamepad input for stage control
            telemetry.addData("arm pos:",arm.getCurrentPosition());
            telemetry.update();
        }
    }
    private void moveToPosition(DcMotor arm, int targetPosition) {
        int currentPosition = arm.getCurrentPosition();

        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5); // Adjust power based on direction


        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", arm.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors after reaching the target;
        arm.setPower(0);
        telemetry.addData("Reached", "Pos " + arm.getCurrentPosition());
        telemetry.update();
    }


}
