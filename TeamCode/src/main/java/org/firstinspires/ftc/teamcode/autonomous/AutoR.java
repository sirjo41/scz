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
                .strafeTo(new Vector2d(32,-15))
                .strafeTo(new Vector2d(45,-15))
                .strafeTo(new Vector2d(45,-50))
                .strafeTo(new Vector2d(45,-15))
                .strafeTo(new Vector2d(55,-15))
                .strafeTo(new Vector2d(55,-50))
                .strafeTo(new Vector2d(55,-15))
                .strafeTo(new Vector2d(61,-15))
                .strafeTo(new Vector2d(61,-50));

        TrajectoryActionBuilder InTakePos = drive.actionBuilder(new Pose2d(61,-50, Math.toRadians(90.00)))
                .strafeToConstantHeading(InTake);

        TrajectoryActionBuilder OutTakePos2 = drive.actionBuilder(new Pose2d(30,-55, Math.toRadians(90.00)))
                .strafeToConstantHeading(OutTake);

        TrajectoryActionBuilder InTakePos2 = drive.actionBuilder(new Pose2d(12,-43, Math.toRadians(90.00)))
                .strafeToConstantHeading(InTake);


        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        OutTakePos.build(),
                        arm.goToStage1(),
                        intakeServos.setWristOutTake()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.goToStage1(),
                        intakeServos.Outtake(),
                        arm.goToStage0()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.goToStage0(),
                        SampToHum.build()
                )
        );
        Actions.runBlocking(
                 new ParallelAction(
                         InTakePos.build(),
                         arm.goToStage1(),
                         intakeServos.setWristOutTake()
                 )
        );
        Actions.runBlocking(
                new SequentialAction(
                        intakeServos.Outtake()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        OutTakePos2.build(),
                        arm.goToStage1(),
                        intakeServos.WOutTake1()
                )
        );

        arm.stop();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}
