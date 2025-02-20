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
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;


@Config
@Autonomous(name = "Specimen Side Auton", group = "Autonomous", preselectTeleOp = "Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(11,-34 );
    public static final Vector2d InTake = new Vector2d(37, -61);

    public static double WRIST_INTAKE = 0.4;

    public static int ARM_DF = 255;

    public static int STAGE_DF = 0;
    public static int STAGE_OUTTAKE = 940;
    public static int STAGE_OUTTAKE2 = 568;

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
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);


        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo wrist = hardwareMap.servo.get("wrist");

        wrist.setPosition(WRIST_INTAKE);

        // Build trajectory segments
        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder InTake2 = OutTake1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(InTake.x,InTake.y),Math.toRadians(270));

        TrajectoryActionBuilder OutTake2 = InTake2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

                TrajectoryActionBuilder SampToHum = OutTake1.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -34))
                .strafeTo(new Vector2d(36, -15))
                .strafeTo(new Vector2d(47, -15))
                .strafeTo(new Vector2d(47, -52))
                .strafeTo(new Vector2d(47, -15))
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(55, -52))
                .strafeTo(new Vector2d(55, -15))
                .strafeTo(new Vector2d(61, -15))
                .strafeTo(new Vector2d(61, -61));

        // Initialize and configure arm motor

        // Signal readiness and wait for start
        telemetry.addData("Status", "READY V2 :) ");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("Status", "Executing Autonomous Routine");
        telemetry.update();

        gotostage(slide1,slide2,STAGE_OUTTAKE);
        Actions.runBlocking(OutTake1.build());
        gotostage2(slide1,slide2,STAGE_OUTTAKE2);
        intake1.setPower(1);
        intake2.setPower(1);
        sleep(1000);
        intake1.setPower(0);
        intake2.setPower(0);


        gotostage(slide1,slide2,STAGE_DF);
        Actions.runBlocking(InTake2.build());
        intake1.setPower(-1);
        intake2.setPower(-1);
        sleep(1000);
        intake1.setPower(0);
        intake2.setPower(0);

        gotostage(slide1,slide2,STAGE_OUTTAKE);
        Actions.runBlocking(OutTake2.build());
        gotostage2(slide1,slide2,STAGE_OUTTAKE2);
        intake1.setPower(1);
        intake2.setPower(1);
        sleep(1000);
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
