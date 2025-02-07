package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(60, 60, Math.PI, Math.PI, 15)
                .setDimensions(18,18)
                .setStartPose(new Pose2d(-68.09, Math.toRadians(90.00)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12.57, -43, Math.toRadians(90.00)))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("put sp"))
//                        .splineToConstantHeading(new Vector2d(35.67, -35.49), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(35.67, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(45.00, -49.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(45.00, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(55.00, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(55.00, -49.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(55.00, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(62.00, -10.00), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(62.00, -49.00), Math.toRadians(90.00))
//                        .splineTo(new Vector2d(35.64,-55.75), Math.toRadians(270.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("take sp"))
//
//                        .splineToLinearHeading(new Pose2d(1.0, -40.0, Math.toRadians(90.00)), Math.toRadians(270.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("put sp"))
//                        .splineToLinearHeading(new Pose2d(35.64,-55.75,Math.toRadians(270.00)), Math.toRadians(90.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("take sp"))
//
//                        .splineToLinearHeading(new Pose2d(1.0, -40.0, Math.toRadians(90.00)), Math.toRadians(270.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("put sp"))
//
//                        .splineToLinearHeading(new Pose2d(35.64,-55.75, Math.toRadians(270.00)), Math.toRadians(90.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("take sp"))
//                        .splineToLinearHeading(new Pose2d(1.0, -40.0,Math.toRadians(90.00)), Math.toRadians(270.00))
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() -> System.out.println("put sp"))
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}