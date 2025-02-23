package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;


@Config
@Autonomous(name = "Specimen Side Auton", group = "Autonomous", preselectTeleOp = "Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(4,-32 );
    public static final Vector2d InTake = new Vector2d(60, -40);

    public static double WRIST_OUTTAKE = 0.4;
    public static double WRIST_INTAKE = 0.7;

    public static int ARM_OUTTAKE = 255;
    public static int ARM_INTAKE = 1500;

    public static int STAGE_DF = 0;
    public static int STAGE_OUTTAKE = 940;
    public static int STAGE_OUTTAKE2 = 568;
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriverRR odo;

    @Override
    public void runOpMode() {
        // Initialize hardware and starting pose

        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14, -61, Math.toRadians(90.0));
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

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(ARM_OUTTAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);

        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo wrist = hardwareMap.servo.get("wrist");

        wrist.setPosition(WRIST_OUTTAKE);

        // Build trajectory segments
        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder InTake2 = OutTake1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(InTake.x,InTake.y),Math.toRadians(270));

        TrajectoryActionBuilder OutTake2 = InTake2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -25),Math.toRadians(90));

        TrajectoryActionBuilder SampToHum = OutTake2.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, -52));

        TrajectoryActionBuilder InTake3 = SampToHum.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(InTake.x,InTake.y),Math.toRadians(270));

        TrajectoryActionBuilder OutTake3 = InTake3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6, -27),Math.toRadians(90));

        // Signal readiness and wait for start
        telemetry.addData("Status", "READY V3 BABYYYYYYYYY MAMAAAMIAAAA :) ");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        telemetry.addData("Status", "Executing Autonomous Routine");
        telemetry.update();

        gotostage(slide1,slide2,STAGE_OUTTAKE);
        Actions.runBlocking(OutTake1.build());
        gotostage2(slide1,slide2,STAGE_OUTTAKE2);
        intake1.setPower(-1);
        intake2.setPower(-1);
        double startTime = runtime.seconds();

        while (opModeIsActive() && (runtime.seconds() - startTime < 1  )) {
            telemetry.addData("Time Elapsed (sec)", runtime.seconds() - startTime);
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);


        gotostage(slide1,slide2,STAGE_DF);
        wrist.setPosition(WRIST_INTAKE);
        Actions.runBlocking(InTake2.build());
        arm.setTargetPosition(ARM_INTAKE);
        while(opModeIsActive()&& arm.isBusy()){

        }
        intake1.setPower(1);
        intake2.setPower(1);
        startTime = runtime.seconds();
        while (opModeIsActive() && (runtime.seconds() - startTime < 1.5)) {
            telemetry.addData("Time Elapsed (sec)", runtime.seconds() - startTime);
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);

        wrist.setPosition(WRIST_OUTTAKE);
        arm.setTargetPosition(ARM_OUTTAKE);
        gotostage(slide1,slide2,STAGE_OUTTAKE);
        Actions.runBlocking(OutTake2.build());
        gotostage2(slide1,slide2,STAGE_OUTTAKE2);
        intake1.setPower(-1);
        intake2.setPower(-1);
        startTime = runtime.seconds();

        while (opModeIsActive() && (runtime.seconds() - startTime < 1)) {
            telemetry.addData("Time Elapsed (sec)", runtime.seconds() - startTime);
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);

        gotostage(slide1,slide2,STAGE_DF);
        Actions.runBlocking(SampToHum.build());


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
