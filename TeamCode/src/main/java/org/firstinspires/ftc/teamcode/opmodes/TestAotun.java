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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.actions.Arm;
import org.firstinspires.ftc.teamcode.actions.IntakeServos;
import  org.firstinspires.ftc.teamcode.actions.Slides;


@Autonomous(name = "TEST",group = "Test")
public class TestAotun extends LinearOpMode {

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Pose2d initialPose = new Pose2d(26.91, -62.25, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder sp = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(4.57, -34.02), Math.toRadians(90.00));

        intakeServos.resetWrist();
        telemetry.addData("Status","DONE");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.goToStage1(),
                        slides.goToStage2(),
                        sp.build()
                )
        );

        arm.holdPosition();

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }
}
