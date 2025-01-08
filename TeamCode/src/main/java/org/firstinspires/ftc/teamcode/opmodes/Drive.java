package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {
    private  static  final int STAGE_0 = 5;
    private static final int STAGE_1 = 900;
    private static final int STAGE_2 = 1800;
    private static final int STAGE_3 = 2800;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

        DcMotor arm = hardwareMap.dcMotor.get("arm");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

            if (gamepad1.a) {
                moveToPosition(slide1, slide2, STAGE_1);
            } else if (gamepad1.b) {
                moveToPosition(slide1, slide2, STAGE_2);
            } else if (gamepad1.y) {
                moveToPosition(slide1, slide2, STAGE_3);
            } else if (gamepad1.x){
                moveToPosition(slide1,slide2,STAGE_0);
            }

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
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2){
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad1.left_trigger);
            }
            else{
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int p = arm.getCurrentPosition();
                arm.setTargetPosition(p);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.2);
            }
        }
    }
    private void moveToPosition(DcMotor slide1, DcMotor slide2, int targetPosition) {
        int currentPosition = slide2.getCurrentPosition();
        int direction = (targetPosition > currentPosition) ? 1 : -1;

        slide2.setTargetPosition(targetPosition);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setPower(1);
        slide1.setPower(direction * 0.7);

        while (opModeIsActive() && slide2.isBusy()) {
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", slide2.getCurrentPosition());
            telemetry.update();
        }

        slide1.setPower(0);
        slide2.setPower(0);
    }
    }
