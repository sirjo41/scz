package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->  drive.trajectorySequenceBuilder(new Pose2d(24.31, -64.55, Math.toRadians(90.00)))
                       .splineToLinearHeading(new Pose2d(4.19, -31.20, Math.toRadians(89.99)), Math.toRadians(89.99)) //FIXME: go to the goaling area
                        .setReversed(true)
                       .splineToLinearHeading(new Pose2d(35.12, -52.44, Math.toRadians(-35.34)), Math.toRadians(-0.37))//FIXME: go to the human area
                        .setReversed(false)
//FIXME: pushing samples
                       .splineToLinearHeading(new Pose2d(35.12, -10.15, Math.toRadians(93.01)), Math.toRadians(93.01))
                        .splineToSplineHeading(new Pose2d(54.30, -7.54, Math.toRadians(7.74)), Math.toRadians(7.74))
                      .splineTo(new Vector2d(62.69, -16.30), Math.toRadians(266.82))
                      .splineToConstantHeading(new Vector2d(61.01, -46.48), Math.toRadians(270.00))

                                .build()


                );

        RoadRunnerBotEntity sec = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->   drive.trajectorySequenceBuilder(new Pose2d(-28.60, -65.11, Math.toRadians(90.00)))

                        .splineToLinearHeading(new Pose2d(-53.93, -37.54, Math.toRadians(118.14)), Math.toRadians(102.53)) //FIXME:  sample basket
                                .splineToLinearHeading(new Pose2d(-52.44, -54.12, Math.toRadians(226.08)), Math.toRadians(226.08)) //fIXME: Basked
                                .build()


                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(sec)
                .start();
    }
}