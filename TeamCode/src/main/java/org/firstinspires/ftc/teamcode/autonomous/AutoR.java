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

@Autonomous(name = "Specimen Side Auton", group = "Autonomous", preselectTeleOp = "Drive")
public class AutoR extends LinearOpMode {

    public static final Vector2d OutTake = new Vector2d(4, -39);
    public static final Vector2d InTake = new Vector2d(40, -59);

    GoBildaPinpointDriverRR odo;

    @Override
    public void runOpMode() {
        // Initialize hardware and starting pose
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90.0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        // Build trajectory segments
        TrajectoryActionBuilder OutTake1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder SampToHum = OutTake1.endTrajectory().fresh()
                .strafeTo(new Vector2d(4, -40))
                .turn(Math.PI / 2 )
                .turn(Math.PI / 2 )
                .strafeTo(new Vector2d(28, -40))
                .strafeTo(new Vector2d(28, -16))
                .strafeTo(new Vector2d(47, -16))
                .strafeTo(new Vector2d(47, -52))
                .strafeTo(new Vector2d(47, -16))
                .strafeTo(new Vector2d(55, -16))
                .strafeTo(new Vector2d(55, -52))
                .strafeTo(new Vector2d(55, -16))
                .strafeTo(new Vector2d(61, -16))
                .strafeTo(new Vector2d(61, -59))
                .waitSeconds(0.5);

        TrajectoryActionBuilder OutTake2 = SampToHum.endTrajectory().fresh()
                .turn(Math.PI )
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y));

        TrajectoryActionBuilder InTake1 = OutTake2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(InTake.x, InTake.y))
                .turn(Math.PI )
                .waitSeconds(0.5);

        TrajectoryActionBuilder OutTake3 = InTake1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(OutTake.x, OutTake.y))
                .turn(Math.PI );

        // Initialize and configure arm motor
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);

        // Signal readiness and wait for start
        telemetry.addData("Status", "READY");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("Status", "Executing Autonomous Routine");
        telemetry.update();

        try {
            // Execute the complete autonomous sequence in one blocking call
            Actions.runBlocking(new SequentialAction(
                    // 1. OutTake1 trajectory with slides to stage 2, then slides to stage 1 and open intake fingers
                    new ParallelAction(
                            slides.goToStage2(),
                            OutTake1.build()
                    ),
                    slides.goToStage1(),
                    intakeServos.openfingers(),
                    // 2. SampToHum trajectory with slides to stage 0, then close intake fingers
                    new ParallelAction(
                            SampToHum.build(),
                            slides.goToStage0()
                    ),
                    intakeServos.closefingers(),

                    // 3. OutTake2 trajectory with slides to stage 2, then slides to stage 1 and open intake fingers
                    new ParallelAction(
                            OutTake2.build(),
                            slides.goToStage2()
                    ),
                    slides.goToStage1(),
                    intakeServos.openfingers(),

                    // 4. First cycle: InTake1 trajectory with slides to stage 0, then close intake fingers,
                    //    OutTake3 trajectory with slides to stage 2, then slides to stage 1 and open intake fingers
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

                    // 5. Second cycle: repeat InTake1 and OutTake3 sequence
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

                    // 6. Third cycle: repeat InTake1 and OutTake3 sequence
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
            ));
        } catch (Exception e) {
            telemetry.addData("Error", "Error executing autonomous routine: " + e.getMessage());
            telemetry.update();

            if (isStopRequested()) return;

            telemetry.addData("Status", "Completed");
            telemetry.update();
        }
    }
}