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
                .setConstraints(80, 70, Math.PI, Math.PI, 12)
                .setDimensions(18,18)
                .setStartPose(new Pose2d(14.25, -62.13, Math.toRadians(90.00)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(14.25, -62.13, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(12.57, -43),Math.toRadians(90.00))
                        .strafeTo(new Vector2d(32,-43))
                        .strafeTo(new Vector2d(32,-18))
                        .strafeTo(new Vector2d(45,-18))
                        .strafeTo(new Vector2d(45,-50))
                        .strafeTo(new Vector2d(45,-18))
                        .strafeTo(new Vector2d(55,-18))
                        .strafeTo(new Vector2d(55,-50))
                        .strafeTo(new Vector2d(55,-18))
                        .strafeTo(new Vector2d(61,-18))
                        .strafeTo(new Vector2d(61,-50))
                        .splineToLinearHeading(new Pose2d(30,-55),Math.toRadians(-80.00))
                        .splineToLinearHeading(new Pose2d(12, -43),Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(30,-55),Math.toRadians(-80.00))
                        .splineToLinearHeading(new Pose2d(12, -43),Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(30,-55),Math.toRadians(-80.00))
                        .splineToLinearHeading(new Pose2d(12, -43),Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(30,-55),Math.toRadians(-80.00))
                        .splineToLinearHeading(new Pose2d(12, -43),Math.toRadians(90.00))
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}