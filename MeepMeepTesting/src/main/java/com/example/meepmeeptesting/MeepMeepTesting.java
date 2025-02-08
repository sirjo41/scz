package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700); //new Pose2d(12.57, -43, Math.toRadians(90.00))

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(1500, 220, Math.PI*2.35, Math.PI, 15)
                .setDimensions(17.5,17.5)
                .setStartPose(new Pose2d(14.25, -62.13, Math.toRadians(90.00)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(14.25, -62.13, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(12.57, -43),Math.toRadians(90.00))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(32,-43))
                        .strafeTo(new Vector2d(32,-15))
                        .strafeTo(new Vector2d(45,-15))
                        .strafeTo(new Vector2d(45,-50))
                        .strafeTo(new Vector2d(45,-15))
                        .strafeTo(new Vector2d(55,-15))
                        .strafeTo(new Vector2d(55,-50))
                        .strafeTo(new Vector2d(55,-15))
                        .strafeTo(new Vector2d(61,-15))
                        .strafeTo(new Vector2d(61,-50))
                        .splineToLinearHeading(new Pose2d(30,-55,Math.toRadians(-90.00)),Math.toRadians(-90.00))
                                .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(10, -40,Math.toRadians(90.00)),Math.toRadians(90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(30,-55,Math.toRadians(-90.00)),Math.toRadians(-90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(8, -40,Math.toRadians(90.00)),Math.toRadians(90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(30,-55,Math.toRadians(-90.00)),Math.toRadians(-90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(6, -40,Math.toRadians(90.00)),Math.toRadians(90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(30,-55,Math.toRadians(-90.00)),Math.toRadians(-90.00))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(4, -40,Math.toRadians(90.00)),Math.toRadians(90.00))
                        .waitSeconds(1)
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
