package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.Arm;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.autonomous.actions.Slides;

@Config
@Autonomous(name = "Right Side Auton",group = "Autonomous",preselectTeleOp ="drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(4, -43);
    public static final Vector2d InTake = new Vector2d(40,-59);
    public static final double turnnn = 3;

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14.25, -62.13, Math.toRadians(90.0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder OutTakePos = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder SampToHum = drive.actionBuilder(new Pose2d(OutTake.x, OutTake.y, Math.toRadians(90.0)))
                .strafeTo(new Vector2d(36,-43))
                .strafeTo(new Vector2d(36,-13))
                .strafeTo(new Vector2d(47,-13))
                .strafeTo(new Vector2d(47,-52))
                .strafeTo(new Vector2d(47,-13))
                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(55,-52))
                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(62,-13))
                .strafeTo(new Vector2d(62,-52))
                .strafeTo(new Vector2d(InTake.x, InTake.y));

//        TrajectoryActionBuilder InTakePos1 = drive.actionBuilder( new Pose2d(OutTake.x,OutTake.y,Math.toRadians(90.00)))
//                .strafeTo(new Vector2d(InTake.x, InTake.y))
//                .turn(Math.PI);
//
//
//        TrajectoryActionBuilder OutTakePos2 = drive.actionBuilder(new Pose2d(InTake.x, InTake.y, Math.toRadians(270.0)))
//                .strafeTo(new Vector2d(OutTake.x, OutTake.y))
//                .turn(Math.PI);
//
//        TrajectoryActionBuilder InTakePos2 = drive.actionBuilder(new Pose2d(OutTake.x, OutTake.y, Math.toRadians(90.0)))
//                .strafeTo(new Vector2d(InTake.x,InTake.y))
//                .turn(Math.PI);


        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        OutTakePos.build(),
                        arm.goToStageOutTake(),
                        intakeServos.setWristOutTake()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        slides.goToStage1(),
                        arm.goToStageIntake2()
                )
        );

        Actions.runBlocking(intakeServos.Outtake());

        Actions.runBlocking(
                new ParallelAction(
                        arm.goToStage0(),
                        slides.goToStage0()
                )
        );

        Actions.runBlocking(SampToHum.build());

       sleep(10000000);

       arm.stop();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}