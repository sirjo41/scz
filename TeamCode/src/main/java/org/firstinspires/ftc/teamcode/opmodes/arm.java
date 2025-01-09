package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "arm", group = "TeleOp")
public class arm extends LinearOpMode {

    // Define constants for stage positions
    private static final int STAGE_0 = 0;
    private static final int STAGE_1 = 550;
    private static final int STAGE_2 = 1600;


    @Override
    public void runOpMode() {
        // Initialize motors
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        // Set the motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check gamepad input for stage control
            if(gamepad1.a){
                moveToPosition(arm,STAGE_0);
            }
            else if(gamepad1.b){
                moveToPosition(arm,STAGE_1);
            }
            else if(gamepad2.y){
                moveToPosition(arm,STAGE_2);
            }
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
        arm.setPower(0.1);
        telemetry.addData("Reached", "Pos " + arm.getCurrentPosition());
        telemetry.update();
    }


}
