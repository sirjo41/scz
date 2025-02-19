package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.Arm;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.autonomous.actions.Slides;

@Config
@Autonomous(name = "Right Side Auton",group = "Autonomous", preselectTeleOp ="Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(4, -45);
    public static final Vector2d InTake = new Vector2d(40,-59);

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14, -62, Math.toRadians(90.0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(OutTake.x, OutTake.y),Math.toRadians(90.00));


        TrajectoryActionBuilder SampToHum = OutTake1.endTrajectory().fresh()
                .turn(Math.PI/2)
                .strafeTo(new Vector2d(33,-45))
                .strafeTo(new Vector2d(33,-14))
                .strafeTo(new Vector2d(47,-14))
                .strafeTo(new Vector2d(47,-52))
                .strafeTo(new Vector2d(47,-14))
                .strafeTo(new Vector2d(55,-14))
                .strafeTo(new Vector2d(55,-52))
                .strafeTo(new Vector2d(55,-14))
                .strafeTo(new Vector2d(61,-14))
                .strafeTo(new Vector2d(61,-56))
                .waitSeconds(0.5);

        TrajectoryActionBuilder OutTake2 = SampToHum.endTrajectory().fresh()
                .turn(Math.PI/2)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));


        TrajectoryActionBuilder InTake1 = OutTake2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(InTake.x, InTake.y))
                .turn(Math.PI/2)
                .waitSeconds(0.5);

        TrajectoryActionBuilder OutTake3 = InTake1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(OutTake.x,OutTake.y))
                .turn(Math.PI/2);


        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                OutTake1.build(),
                                arm.goToStage0(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers()
                )
        ); // FIRST THING puts specmien

//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage0(),
//                                SampToHum.build()
//                        ),
//                        intakeServos.closefingers()
//                )
//        ); // SECOND THING go to human player and get the specmien
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage2(),
//                                OutTake2.build()
//                        ),
//                        slides.goToStage1(),
//                        intakeServos.openfingers()
//                )
//        ); // OUT TAKE SECOND ONE
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage0(),
//                                InTake1.build()
//                        ),
//                        intakeServos.closefingers()
//                )
//        ); // INTAKE SAMPLE
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage2(),
//                                OutTake3.build()
//                        ),
//                        slides.goToStage1(),
//                        intakeServos.openfingers()
//                )
//        ); // outake 3
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage0(),
//                                InTake1.build()
//                        ),
//                        intakeServos.closefingers()
//                )
//        ); // intake sample
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage2(),
//                                OutTake3.build()
//                        ),
//                        slides.goToStage1(),
//                        intakeServos.openfingers()
//                )
//        ); // outake 4
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage0(),
//                                InTake1.build()
//                        ),
//                        intakeServos.closefingers()
//                )
//        ); // intake sample
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                slides.goToStage2(),
//                                OutTake3.build()
//                        ),
//                        slides.goToStage1(),
//                        intakeServos.openfingers()
//                )
//        ); // outake 5


        sleep(1000000);
       arm.stop();



        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}