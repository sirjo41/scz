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

@Autonomous(name = "Left Side Auton",group = "Autonomous",preselectTeleOp ="drive")
public class AutoL extends LinearOpMode {

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose =new Pose2d(-13.76, -61.76, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder InTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-48, -42));;

        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(new Pose2d(-48.30, -42.88, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-54,-54))
                .turn(2.4)
                .turn(-2.4);

        TrajectoryActionBuilder InTake2 = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-58, -42.88));

        TrajectoryActionBuilder InTake3 = drive.actionBuilder(new Pose2d(-54, -42.88, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-52, -24))
                .turn(1.555);
        TrajectoryActionBuilder OutTake2 = drive.actionBuilder(new Pose2d(-52,-24, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-54,-54))
                .turn(0.8)
                .turn(-0.8);

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-54,-54, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-47,-60));


        telemetry.addData("Status","READDDYYYY BABBYYYYYYY ");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                InTake1.build()
        );
        Actions.runBlocking(
                new ParallelAction(
                        arm.goToStageInTake3(),
                        intakeServos.setWristOutDf()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                slides.goToStage1(),
                intakeServos.Intake(),
                new ParallelAction(
                        slides.goToStage0(),
                        arm.goToStage0()
                )
        ));

        Actions.runBlocking(
                new ParallelAction(
                OutTake1.build(),
                slides.goToStage2(),
                intakeServos.setWristInTake(),
                intakeServos.Outtake(),
                intakeServos.setWristOutTake(),
                slides.goToStage0()
                )
        );

        Actions.runBlocking(
                park.build()
        );




        arm.stop();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}