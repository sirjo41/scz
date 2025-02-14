package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathSegment;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static final Vector2d OutTake = new Vector2d(4, -43);
    public static final Vector2d InTake = new Vector2d(40,-56);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(60, 60, Math.PI, Math.PI, 15)
                .setDimensions(18,18)
                .setStartPose(new Pose2d(-68.09, Math.toRadians(90.00)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-4, -43, Math.toRadians(90.0)))
                        .strafeTo(new Vector2d(36,-43))
                        .strafeTo(new Vector2d(36,-13))
                        .strafeTo(new Vector2d(47,-13))
                        .strafeTo(new Vector2d(47,-52))
                        .strafeTo(new Vector2d(47,-13))
                        .strafeTo(new Vector2d(55,-13))
                        .strafeTo(new Vector2d(55,-52))
                        .strafeTo(new Vector2d(55,-13))
                        .strafeTo(new Vector2d(62,-13))
                        .strafeTo(new Vector2d(62,-52))
                        .strafeTo(new Vector2d(40,-56))
                        .turn(3.15)
                        .turn(3.15)
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}