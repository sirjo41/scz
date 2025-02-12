package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathSegment;

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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-13.76, -61.76, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(-48.30, -42.88),Math.toRadians(90.00))
                        .splineTo(new Vector2d(-48.30, -42.88),Math.toRadians(221.09))
                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}