package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {
    private static final int A_STAGE_0 = 0;
    private static final int A_STAGE_1 = 550;
    private static final int A_STAGE_2 = 1600;

    private static final double INTAKE_OPEN_POSITION = 1.0;
    private static final double INTAKE_CLOSED_POSITION = 0.0;

    private static final double RS_DEFAULT = 0.8;
    private static final double RS_CLOSED = 0.0;
    private static final double RS_OPEN = 1.0;

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

        Servo intakeServo1 = hardwareMap.servo.get("intakeServo1");
        Servo intakeServo2 = hardwareMap.servo.get("intakeServo2");
        Servo rsServo = hardwareMap.servo.get("rs");

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

        // Set Initial Servo Positions
        intakeServo1.setPosition(INTAKE_CLOSED_POSITION);
        intakeServo2.setPosition(INTAKE_OPEN_POSITION);
        rsServo.setPosition(RS_DEFAULT);

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

            if (gamepad1.right_bumper) {
                slide1.setPower(0.7);
                slide2.setPower(1);
            } else if (gamepad1.left_bumper) {
                slide1.setPower(-0.7);
                slide2.setPower(-1);
            } else {
                slide1.setPower(0);
                slide2.setPower(0);
            }

            if (gamepad1.right_trigger > 0.2) {
                arh = true;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2) {
                arh = true;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad1.left_trigger);
            } else {
                if (arm.getCurrentPosition() <= 200 && arm.getCurrentPosition() >= -200) {
                    arm.setPower(0);
                    arh = false;
                }
                if (arh) {
                    holdPosition(arm);
                    arh = false;
                }
            }

            if (gamepad1.dpad_up) {
                moveArmToPosition(arm, A_STAGE_0);
            }
            if (gamepad1.dpad_right) {
                moveArmToPosition(arm, A_STAGE_1);
            }
            if (gamepad1.dpad_down) {
                moveArmToPosition(arm, A_STAGE_2);
            }

            // Servo Controls
            if (gamepad1.a) {
                moveIntakeServos(intakeServo1, intakeServo2, INTAKE_OPEN_POSITION, INTAKE_CLOSED_POSITION);
            } else if (gamepad1.b) {
                moveIntakeServos(intakeServo1, intakeServo2, INTAKE_CLOSED_POSITION, INTAKE_OPEN_POSITION);
            }

            if (gamepad1.x) {
                rsServo.setPosition(RS_CLOSED);
            } else if (gamepad1.y) {
                rsServo.setPosition(RS_OPEN);
            }
        }
    }

    private void moveArmToPosition(DcMotor arm, int targetPosition) {
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);

        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0.1);
    }

    private void holdPosition(DcMotor arm) {
        int currentTarget = arm.getCurrentPosition();
        arm.setTargetPosition(currentTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.1);
    }

    private void moveIntakeServos(Servo intakeServo1, Servo intakeServo2, double position1, double position2) {
        intakeServo1.setPosition(position1);
        intakeServo2.setPosition(position2);

        telemetry.addData("Intake Servo 1", position1);
        telemetry.addData("Intake Servo 2", position2);
        telemetry.update();
    }
}
