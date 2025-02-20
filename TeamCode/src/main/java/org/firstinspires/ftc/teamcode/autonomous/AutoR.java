package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;


@Config
@Autonomous(name = "Specimen Side Auton", group = "Autonomous", preselectTeleOp = "Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(2, -40);
    public static final Vector2d InTake = new Vector2d(40, -65);

    public static double FINGERS_OPEN = 0.4;
    public static double FINGERS_CLOSE = 0.6;
    public static double ELBOW_INTAKE = 0.8;
    public static double SHOULDER_INTAKE = 1;
    public static double WRIST_INTAKE = 1;

    public static int STAGE_INTAKE = 5;
    public static int STAGE_OUTTAKE = 1500;
    public static int STAGE_OUTTAKE2 = 2200;

    GoBildaPinpointDriverRR odo;

    @Override
    public void runOpMode() {
        // Initialize hardware and starting pose
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90.0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        DcMotor slide1 = hardwareMap.get(DcMotor.class, "slide1");
        DcMotor slide2 = hardwareMap.get(DcMotor.class, "slide2");

        // Set motor directions
        slide1.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo fingers = hardwareMap.servo.get("fingers");
        Servo elbow  = hardwareMap.servo.get("elbow");
        Servo shoulder = hardwareMap.servo.get("shoulder");
        Servo wrist = hardwareMap.servo.get("wrist");

//        // Stop all servos initially
        fingers.setPosition(FINGERS_CLOSE);
        elbow.setPosition(ELBOW_INTAKE);
        shoulder.setPosition(SHOULDER_INTAKE);
        wrist.setPosition(WRIST_INTAKE);

        // Build trajectory segments
        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder SampToHum = OutTake1.endTrajectory().fresh()
                .strafeTo(new Vector2d(26, -45))
                .strafeTo(new Vector2d(26, -20))
                .strafeTo(new Vector2d(47, -20))
                .strafeTo(new Vector2d(47, -52))
                .strafeTo(new Vector2d(47, -20))
                .strafeTo(new Vector2d(55, -20))
                .strafeTo(new Vector2d(55, -52))
                .strafeTo(new Vector2d(55, -20))
                .strafeTo(new Vector2d(61, -20))
                .strafeTo(new Vector2d(61, -59))
                .turn(Math.PI /2)
                .turn(Math.PI /2);

        TrajectoryActionBuilder OutTake2 = SampToHum.endTrajectory().fresh()
                .turn(Math.PI / 2)
                .turn(Math.PI / 2)
                .strafeTo(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder InTake2 = OutTake1.endTrajectory().fresh()
                .strafeTo(new Vector2d(InTake.x, InTake.y))
                .turn(Math.PI / 2)
                .turn(Math.PI / 2);

        TrajectoryActionBuilder InTake1 = OutTake2.endTrajectory().fresh()
                .strafeTo(new Vector2d(InTake.x, InTake.y))
                .turn(Math.PI / 2)
                .turn(Math.PI / 2)
                .waitSeconds(0.3);

        TrajectoryActionBuilder OutTake3 = InTake1.endTrajectory().fresh()
                .turn(Math.PI / 2)
                .turn(Math.PI / 2)
                .strafeTo(new Vector2d(OutTake.x, OutTake.y));

        // Initialize and configure arm motor
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);

        // Signal readiness and wait for start
        telemetry.addData("Status", "READY V2 :) ");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("Status", "Executing Autonomous Routine");
        telemetry.update();



            if (isStopRequested()) return;

            telemetry.addData("Status", "Completed");
            telemetry.update();
        }
    private void gotostage(DcMotor slide1, DcMotor slide2, int targ) {
        slide1.setTargetPosition(targ);
        slide2.setTargetPosition(targ);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1);
        slide2.setPower(1);
    }
    private void gotostage2(DcMotor slide1, DcMotor slide2, int targ) {
        slide1.setTargetPosition(targ);
        slide2.setTargetPosition(targ);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(1);
        slide2.setPower(1);
        while (opModeIsActive()&& (slide1.isBusy() && slide2.isBusy())){

        }
        slide1.setPower(0);
        slide2.setPower(0);
    }
    }
