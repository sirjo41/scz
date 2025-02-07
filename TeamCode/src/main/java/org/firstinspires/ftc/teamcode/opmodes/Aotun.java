package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.actions.Arm;
import org.firstinspires.ftc.teamcode.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.actions.Slides;

@Disabled
@Autonomous(name = "AutonT", group = "Test")
public class Aotun extends LinearOpMode {

    GoBildaPinpointDriverRR odo;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);

        Pose2d initialPose = new Pose2d(24.31, -68.09, Math.toRadians(90.00));
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

        TrajectoryActionBuilder sp3 = drive.actionBuilder(new Pose2d(1.0, -40.0, Math.toRadians(90.00)))
                .splineTo(new Vector2d(35.64, -55.75), Math.toRadians(270.00));

        TrajectoryActionBuilder sp4 = drive.actionBuilder(new Pose2d(35.64, -55.75, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(1.0, -40.0), Math.toRadians(90.00));

        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        sp.build(),            // Drive to first position
                        slides.goToStage2(),   // Move slides to position
                        arm.goToStage1(),      // Move arm to position
                        intakeServos.openIntake() // Open intake for object pickup
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        intakeServos.openIntake(), // Activate intake roller forward
                        sp2.build(),                 // Drive trajectory for pickup/delivery
                        intakeServos.closeIntake(),   // Stop intake
                        arm.goToStage1(),            // Arm movement
                        slides.goToStage2(),         // Move slides
                        intakeServos.closeIntake(),  // Close intake to secure object
                        sp4.build()                  // Drive back to starting position
                )
        );

        telemetry.addData("Status", "Autonomous Completed");
        telemetry.update();
    }
}
