package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Aotun extends LinearOpMode {

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                        .turn(90)
                        .waitSeconds(1)
                        .turn(90);

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );
    }
}
