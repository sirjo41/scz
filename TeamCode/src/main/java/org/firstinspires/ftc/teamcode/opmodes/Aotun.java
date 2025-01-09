package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
@Autonomous(name = "AutonT",group = "Test",preselectTeleOp = "Drive")
public class Aotun extends LinearOpMode {

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Slides slides = new Slides(hardwareMap);


        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(10.53, 14.81, Math.toRadians(132.17)), Math.toRadians(132.17));

        telemetry.addData("Status","DONE");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                        tab1.build()
        );
    }
}
