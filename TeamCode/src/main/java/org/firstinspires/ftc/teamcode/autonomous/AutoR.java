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

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.Arm;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.autonomous.actions.Slides;

@Autonomous(name = "Right Side Auton",group = "Autonomous")
public class AutoR extends LinearOpMode {

    private static final Vector2d OutTake = new Vector2d(12, -43);
    private static final Vector2d InTake = new Vector2d(30,-55);
    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14.25, -62.13, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder OutTakePos = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(OutTake);

        TrajectoryActionBuilder SampToHum = drive.actionBuilder(new Pose2d(12.57, -43, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(32,-43))
                .strafeTo(new Vector2d(32,-14))
                .strafeTo(new Vector2d(47,-14))
                .strafeTo(new Vector2d(47,-52))
                .strafeTo(new Vector2d(47,-14))
                .strafeTo(new Vector2d(55,-14))
                .strafeTo(new Vector2d(55,-52))
                .strafeTo(new Vector2d(55,-14))
                .strafeTo(new Vector2d(61,-14))
                .strafeTo(new Vector2d(61,-52));

        TrajectoryActionBuilder InTakePos = drive.actionBuilder(new Pose2d(61,-52, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(30,-52));

        TrajectoryActionBuilder OutTakePos2 = drive.actionBuilder(new Pose2d(30,-52, Math.toRadians(90.00)))
                .strafeTo(OutTake);

        TrajectoryActionBuilder InTakePos2 = drive.actionBuilder(new Pose2d(12,-43, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(30,-52));


        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        OutTakePos.build(),
                        arm.goToStage2(),
                        intakeServos.setWristOutTake()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.goToStage1(),
                        intakeServos.Outtake()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        arm.goToStage0(),
                        slides.goToStage0(),
                        SampToHum.build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        InTakePos.build(),
                        arm.goToStage1(),
                        intakeServos.setWristInTake()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        intakeServos.Intake()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        OutTakePos2.build(),
                        arm.goToStage2(),
                        intakeServos.WOutTake1()
                )
        );

        arm.stop();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}