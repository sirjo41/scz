package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {
    private static final int A_STAGE_0 = 0;
    private static final int A_STAGE_1 = 550;
    private static final int A_STAGE_2 = 1600;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean arh = false;

        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

        DcMotor arm = hardwareMap.dcMotor.get("arm");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.right_bumper){
                slide1.setPower(0.7);
                slide2.setPower(1);
            }
            else if(gamepad1.left_bumper){
                slide1.setPower(-0.7);
                slide2.setPower(-1);
            }
            else{
                slide1.setPower(0);
                slide2.setPower(0);
            }

            if(gamepad1.right_trigger > 0.2){
                arh = true;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2){
                arh = true;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad1.left_trigger);
            }
            else{
                if(arm.getCurrentPosition() <= 200 && arm.getCurrentPosition() >= -200){
                    arh=false;
                }
                if(arh){
                    holdp(arm);
                    arh = false;
                }
            }
            if(gamepad1.dpad_up){
                ArmmoveToPosition(arm, A_STAGE_0);
            }
            if(gamepad1.dpad_right){
                ArmmoveToPosition(arm, A_STAGE_1);
            }
            if(gamepad1.dpad_down){
                ArmmoveToPosition(arm, A_STAGE_2);
            }
            //TODO: add the intake... :)
//            if(gamepad1.left_stick_button){
//
//            }
//            else if(gamepad1.right_stick_button){
//
//            }
        }
    }

    private void ArmmoveToPosition(DcMotor arm, int targetPosition) {
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

    private  void holdp(DcMotor arm){
        int currentTarget = arm.getCurrentPosition();
        arm.setTargetPosition(currentTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.1);
    }
    }
