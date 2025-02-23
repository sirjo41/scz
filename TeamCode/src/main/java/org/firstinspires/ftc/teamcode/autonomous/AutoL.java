package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name = "Basket Side Auton",group = "Autonomous",preselectTeleOp ="Drive")
public class AutoL extends LinearOpMode {

    public static final Vector2d OutTakeSub = new Vector2d(-11,-22);
    public static double WRIST_OUTTAKE = 0.4;
    public static double WRIST_INTAKE = 0.7;

    public static int ARM_OUTTAKESUB = 255;
    public static int ARM_INTAKE = 1776;

    public static int STAGE_DF = 0;
    public static int STAGE_OUTTAKESUB = 940;
    public static int STAGE_OUTTAKE2SUB = 568;
    private ElapsedTime runtime = new ElapsedTime();
    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(-34, -61, Math.toRadians(90.0));
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
        arm.setTargetPosition(ARM_OUTTAKESUB);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);

        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo wrist = hardwareMap.servo.get("wrist");

        wrist.setPosition(WRIST_OUTTAKE);


        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTakeSub.x, OutTakeSub.y));

        TrajectoryActionBuilder park = OutTake1.endTrajectory().fresh()
                        .strafeToConstantHeading(new Vector2d(-53,-61));



        telemetry.addData("Status","READY :( ");
        telemetry.update();

        waitForStart();
        gotostage(slide1,slide2, STAGE_OUTTAKESUB);
        Actions.runBlocking(OutTake1.build());
        gotostage2(slide1,slide2, STAGE_OUTTAKE2SUB);
        intake1.setPower(-1);
        intake2.setPower(-1);
        double startTime = runtime.seconds();

        while (opModeIsActive() && (runtime.seconds() - startTime < 1)) {
            telemetry.addData("Time Elapsed (sec)", runtime.seconds() - startTime);
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);

        gotostage(slide1,slide2,STAGE_DF);

        Actions.runBlocking(park.build());


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