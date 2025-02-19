package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.autonomous.actions.Slides;

@Autonomous(name = "Specimen Side Auton",group = "Autonomous", preselectTeleOp ="Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(4, -39);
    public static final Vector2d InTake = new Vector2d(40,-59);

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14, -62, Math.toRadians(90.0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        //Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));


        TrajectoryActionBuilder SampToHum = OutTake1.endTrajectory().fresh()
                .strafeTo(new Vector2d(4,-40))
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
                .strafeTo(new Vector2d(61,-59))
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
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        // Initialize PID Controller

        // Set motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();

        arm.setPower(0.9);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                OutTake1.build(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers(),
                        new ParallelAction(
                                SampToHum.build(),
                                slides.goToStage0()
                        ),
                        intakeServos.closefingers(),
                        new ParallelAction(
                                OutTake2.build(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers(),
                        new ParallelAction(
                                InTake1.build(),
                                slides.goToStage0()
                        ),
                        intakeServos.closefingers(),
                        new ParallelAction(
                                OutTake3.build(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers(),
                        new ParallelAction(
                                InTake1.build(),
                                slides.goToStage0()
                        ),
                        intakeServos.closefingers(),
                        new ParallelAction(
                                OutTake3.build(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers(),
                        new ParallelAction(
                                InTake1.build(),
                                slides.goToStage0()

                        ),
                        intakeServos.closefingers(),
                        new ParallelAction(
                                OutTake3.build(),
                                slides.goToStage2()
                        ),
                        slides.goToStage1(),
                        intakeServos.openfingers()
                )
        );
        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }


}