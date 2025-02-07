package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.actions.Arm;
import org.firstinspires.ftc.teamcode.actions.IntakeServos;
import  org.firstinspires.ftc.teamcode.actions.Slides;


@Autonomous(name = "TEST",group = "Test")
public class TestAotun extends LinearOpMode {
    private static final int S_DF = 0;
    private static final int S_OUTTAKE = 800;

    private static final int A_DF = 0;
    private static final int A_1 = 800;
    private static final int A_2 = 1600;
    private static final int A_3 = 2200;

    private static final double WR_DF = 0.5;
    private static final double WR_CLOSED = 0.0;
    private static final double WR_OPEN = 1.0;

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(26.91, -62.25, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder sp = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(1.0, -40.0), Math.toRadians(90.00));

        TrajectoryActionBuilder sp2 = drive.actionBuilder(new Pose2d(1.0, -40.0, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(35.67, -35.49), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(35.67, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(45.00, -49.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(55.00, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(55.00, -49.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(55.00, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(62.00, -10.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(62.00, -49.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(35.64, -55.75), Math.toRadians(270.00));

        DcMotor arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor slide1 = hardwareMap.dcMotor.get("slide1");
        DcMotor slide2 = hardwareMap.dcMotor.get("slide2");
        Servo wrist = hardwareMap.servo.get("Wrist");

        wrist.setPosition(WR_DF);
        moveArmTo(arm,A_DF);
        moveSlideToPos(slide1,slide2,S_DF);

        telemetry.addData("Status","INT");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }

    private void moveArmTo(DcMotor arm,int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

    }

    private  void  moveSlideToPos(DcMotor slides1, DcMotor slides2,int pos){
        slides1.setTargetPosition(pos);
        slides2.setTargetPosition(pos);
        slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides1.setPower(0.7);
        slides2.setPower(1);

        do {
            telemetry.addData("Target", pos);
            telemetry.addData("Current Position Slide1", slides1.getCurrentPosition());
            telemetry.addData("Current Position Slide2", slides2.getCurrentPosition());
            telemetry.update();
        } while (opModeIsActive() && slides1.isBusy() && slides2.isBusy());
        slides1.setPower(0);
        slides2.setPower(0);
    }
}
