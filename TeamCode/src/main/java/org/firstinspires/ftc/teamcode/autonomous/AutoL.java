package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.autonomous.actions.Arm;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.autonomous.actions.Slides;

@Autonomous(name = "Basket Side Auton",group = "Autonomous",preselectTeleOp ="Drive")
public class AutoL extends LinearOpMode {

    public static final Vector2d OutTakeSub = new Vector2d(-11,-32 );
    public static double WRIST_OUTTAKE = 0.4;
    public static double WRIST_INTAKE = 0.7;

    public static int ARM_OUTTAKE = 255;
    public static int ARM_INTAKE = 1402;

    public static int STAGE_DF = 0;
    public static int STAGE_OUTTAKE = 940;
    public static int STAGE_OUTTAKE2 = 568;
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
        arm.setTargetPosition(ARM_OUTTAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);

        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo wrist = hardwareMap.servo.get("wrist");

        wrist.setPosition(WRIST_OUTTAKE);


        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTakeSub.x, OutTakeSub.y));


//        TrajectoryActionBuilder InTake1 = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(new Vector2d(-48, -42));;
//
//        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(new Pose2d(-48.30, -42.88, Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(-54,-54))
//                .turn(2.4)
//                .turn(-2.4);
//
//        TrajectoryActionBuilder InTake2 = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(-58, -42.88));
//
//        TrajectoryActionBuilder InTake3 = drive.actionBuilder(new Pose2d(-54, -42.88, Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(-52, -24))
//                .turn(1.555);
//        TrajectoryActionBuilder OutTake2 = drive.actionBuilder(new Pose2d(-52,-24, Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(-54,-54))
//                .turn(0.8)
//                .turn(-0.8);



        telemetry.addData("Status","READY :( ");
        telemetry.update();

        waitForStart();
        gotostage(slide1,slide2,STAGE_OUTTAKE);
        Actions.runBlocking(OutTake1.build());
        gotostage2(slide1,slide2,STAGE_OUTTAKE2);
        intake1.setPower(-1);
        intake2.setPower(-1);
        double startTime = runtime.seconds();

        while (opModeIsActive() && (runtime.seconds() - startTime < 1)) {
            telemetry.addData("Time Elapsed (sec)", runtime.seconds() - startTime);
            telemetry.update();
        }
        intake1.setPower(0);
        intake2.setPower(0);


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