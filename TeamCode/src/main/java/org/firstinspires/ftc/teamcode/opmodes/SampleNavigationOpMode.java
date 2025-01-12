package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Sample Navigation", group = "Auton")
public class SampleNavigationOpMode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private FtcDashboard dashboard;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor arm;
    private DcMotor slide1;
    private DcMotor slide2;

    private static final int STAGE_1 = 900;
    private static final int STAGE_2 = 1800;
    private static final int STAGE_3 = 2800;

    @Override
    public void runOpMode() {
        // Initialize hardware
        dashboard = FtcDashboard.getInstance();
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        arm = hardwareMap.dcMotor.get("arm");
        slide1 = hardwareMap.dcMotor.get("slide1");
        slide2 = hardwareMap.dcMotor.get("slide2");

        // Set motors to brake when not powered
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Initialize the camera and pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SampleDetectionPipeline pipeline = new SampleDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error:", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            String detectedColor = pipeline.getDetectedColor();
            String direction = pipeline.getDirection();
            double size = pipeline.size;

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Direction", direction);
            telemetry.addData("Size", size);
            telemetry.update();

            if (!detectedColor.equals("None")) {
                moveToSample(direction);
                adjustArmAndSlides(size);
                break; // Stop after processing the detected sample
            }

            sleep(100); // Allow time for processing
        }

        webcam.stopStreaming();
    }

    private void moveToSample(String direction) {
        // Move the robot based on the direction to align with the sample
        switch (direction) {
            case "Left":
                driveMecanum(-0.1, 0, 0); // Strafe left
                break;
            case "Right":
                driveMecanum(0.1, 0, 0); // Strafe right
                break;
            case "Forward":
                driveMecanum(0, 0.1, 0); // Move forward
                break;
            case "Back":
                driveMecanum(0, -0.1, 0); // Move backward
                break;
            case "Done":
                stopMotors();
                return; // Stop if aligned
        }
        sleep(200); // Allow time for adjustment
        stopMotors();
    }

    private void adjustArmAndSlides(double size) {
        // Adjust the arm and slides based on the sample size
        int armPosition = (int) (size / 10); // Example scaling for arm position
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.1);

        int slidePosition;
        if (size < 1000) {
            slidePosition = STAGE_1;
        } else if (size < 2000) {
            slidePosition = STAGE_2;
        } else {
            slidePosition = STAGE_3;
        }

        slide1.setTargetPosition(slidePosition);
        slide2.setTargetPosition(slidePosition);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(0.1);
        slide2.setPower(0.1);

        while (arm.isBusy() || slide1.isBusy() || slide2.isBusy()) {
            // Wait until adjustments are complete
        }

        stopMotors();
    }

    private void driveMecanum(double strafe, double forward, double rotate) {
        double flPower = forward + strafe + rotate;
        double blPower = forward - strafe + rotate;
        double frPower = forward - strafe - rotate;
        double brPower = forward + strafe - rotate;

        frontLeftMotor.setPower(flPower);
        backLeftMotor.setPower(blPower);
        frontRightMotor.setPower(frPower);
        backRightMotor.setPower(brPower);
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
