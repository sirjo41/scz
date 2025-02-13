package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {
    private boolean isHolding = false;
    private Thread holdThread;

    private  static  final  int S_INTAKE = 800;

    private static final double WR_DF = 0.7;
    private static final double WR_CLOSED = 0.2;
    private static final double WR_OPEN = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        //define motors
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");

        DcMotor arm = hardwareMap.dcMotor.get("arm");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        CRServo intakeServo1 = hardwareMap.crservo.get("Lin");
        CRServo intakeServo2 = hardwareMap.crservo.get("Rin");
        CRServo hook1 = hardwareMap.crservo.get("hook1");
        CRServo hook2 = hardwareMap.crservo.get("hook2");
        Servo wrist = hardwareMap.servo.get("Wrist");

        //motors directions
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo2.setDirection(DcMotorSimple.Direction.REVERSE);

        //motors modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motors zero power behavior
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        // Set Initial Servo Positions
//        intakeServo1.setPosition(INTAKE_CLOSED_POSITION);
//        intakeServo2.setPosition(INTAKE_OPEN_POSITION);
        wrist.setPosition(WR_DF);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //drivetrain
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * 1.1;
            double rx = gamepad2.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //slides
            if (gamepad1.right_bumper) {
                slide1.setPower(1);
                slide2.setPower(0.7);
            } else if (gamepad1.left_bumper) {
                slide1.setPower(-1);
                slide2.setPower(-0.7);
            } else {
                slide1.setPower(0);
                slide2.setPower(0);
            }

            //arm

        if(gamepad1.left_stick_y >= 0.2 || gamepad1.left_stick_y <= 0.2){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(gamepad1.left_stick_y);
            }
            else if (arm.getCurrentPosition() <= 200 && arm.getCurrentPosition() >= -200) {
                    arm.setPower(0);
                }
            else {
                    holdPosition(arm);
                }
//wrist 

            if (gamepad1.dpad_up) {
                wrist.setPosition(WR_OPEN);
            }
            if (gamepad1.dpad_right) {
                wrist.setPosition(WR_DF);
            }
            if (gamepad1.dpad_down) {
                wrist.setPosition(WR_CLOSED);
            }

            //servos
            if (gamepad1.square) {
                intakeServo1.setPower(1);
                intakeServo2.setPower(1);
            }
            else if(gamepad1.circle){
                intakeServo1.setPower(-1);
                intakeServo2.setPower(-1);
            }
            else {
                intakeServo1.setPower(0);
                intakeServo2.setPower(0);
            }
            //hook
            if (gamepad2.dpad_down) {
                hook1.setPower(1);
                hook2.setPower(1);
            }
            else if(gamepad1.dpad_up){
                hook1.setPower(-1);
                hook2.setPower(-1);
            }
            else {
                hook1.setPower(0);
                hook2.setPower(0);
            }


        }
    }

private void holdPosition(DcMotor arm) {
        int currentTarget = arm.getCurrentPosition();
        arm.setTargetPosition(currentTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);
    }

}