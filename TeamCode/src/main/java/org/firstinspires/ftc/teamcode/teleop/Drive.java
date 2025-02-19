package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

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

        CRServo wrist = hardwareMap.crservo.get("wrist"); // no need;
        Servo fingers = hardwareMap.servo.get("fingers");
        CRServo hook1 = hardwareMap.crservo.get("hook1");
        CRServo hook2 = hardwareMap.crservo.get("hook2");
        Servo elbow = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");

        //motors directions
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
                slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide1.setPower(1);
                slide2.setPower(1);
            } else if (gamepad1.left_bumper) {
                slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide1.setPower(-1);
                slide2.setPower(-1);
            }
            else if(gamepad1.dpad_left){
                slide1.setTargetPosition(800);
                slide2.setTargetPosition(800);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slide1.setPower(1);
                slide2.setPower(1);

                while(opModeIsActive() && (slide1.isBusy()&& slide1.isBusy())){
                    telemetry.addData("slide1 pos",slide1.getCurrentPosition());
                    telemetry.addData("slide2 pos",slide2.getCurrentPosition());
                    telemetry.update();
                }

                slide1.setPower(0);
                slide2.setPower(0);
                slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else {
                holdPosition(slide1);
                holdPosition(slide2);
            }

            //arm
            if(Math.abs(gamepad1.left_stick_y) >= 0.1){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(gamepad1.left_stick_y);
            }
            else {
                holdPosition(arm);
            }


            //shoulder
            if (gamepad1.dpad_up) {
                shoulder.setPosition(0);
                elbow.setPosition(0.2);
            }
            if (gamepad1.dpad_right) {
                shoulder.setPosition(0.5);
                elbow.setPosition(0.6);
            }
            if (gamepad1.dpad_down) {
                shoulder.setPosition(1);
                elbow.setPosition(0.8);
            }
//fingers
            if(gamepad1.b){
                fingers.setPosition(0.4); // close
            }
            else if (gamepad1.x){
                fingers.setPosition(0.6); // open
            }
            //wrist
            if (gamepad1.y) {
                wrist.setPower(0.5);
                wrist.setPower(0.5);
            }
            else if(gamepad1.a){
                wrist.setPower(-0.5);
                wrist.setPower(-0.5);
            }
            else {
                wrist.setPower(0);
                wrist.setPower(0);
            }


            //hook
            if (gamepad2.dpad_down) {
                hook1.setPower(1);
                hook2.setPower(-1);
            }
            else if(gamepad2.dpad_up){
                hook1.setPower(-1);
                hook2.setPower(1);
            }
            else {
                hook1.setPower(0);
                hook2.setPower(0);
            }


        }
    }

private void holdPosition(DcMotor motor) {
        int currentTarget = motor.getCurrentPosition();
    motor.setTargetPosition(currentTarget);
    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor.setPower(0.7);
    }

}