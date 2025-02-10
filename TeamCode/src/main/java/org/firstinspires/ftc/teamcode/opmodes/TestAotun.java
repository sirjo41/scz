package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.actions.Arm;
import org.firstinspires.ftc.teamcode.actions.IntakeServos;
import org.firstinspires.ftc.teamcode.actions.Slides;

import kotlin.math.MathKt;

@Autonomous(name = "TEST",group = "Test")
public class TestAotun extends LinearOpMode {

    private static final Vector2d OutTake = new Vector2d(12, -43);
    private static final Vector2d InTake = new Vector2d(30,-55);
    private static final int S_DF = 0;
    private static final int S_OUTTAKE = 800;

    private static final int A_DF = 0;
    private static final int A_1 = 800;
    private static final int A_2 = 1600;
    private static final int A_3 = 2200;

    private static final double WR_DF = 0.5;
    private static final double WR_CLOSED = 0.0;
    private static final double WR_OPEN = 1.0;

    GoBildaPinpointDriverRR odo;
    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        Pose2d initialPose = new Pose2d(14.25, -62.13, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        TrajectoryActionBuilder OutTakePos = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(OutTake);

        TrajectoryActionBuilder SampToHum = drive.actionBuilder(new Pose2d(12.57, -43, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(32,-43))
                .strafeTo(new Vector2d(32,-15))
                .strafeTo(new Vector2d(45,-15))
                .strafeTo(new Vector2d(45,-50))
                .strafeTo(new Vector2d(45,-15))
                .strafeTo(new Vector2d(55,-15))
                .strafeTo(new Vector2d(55,-50))
                .strafeTo(new Vector2d(55,-15))
                .strafeTo(new Vector2d(61,-15))
                .strafeTo(new Vector2d(61,-50));

        TrajectoryActionBuilder InTakePos = drive.actionBuilder(new Pose2d(61,-50, Math.toRadians(90.00)))
                .strafeToConstantHeading(InTake);

        TrajectoryActionBuilder OutTakePos2 = drive.actionBuilder(new Pose2d(30,-55, Math.toRadians(90.00)))
                .strafeToConstantHeading(OutTake);

        TrajectoryActionBuilder InTakePos2 = drive.actionBuilder(new Pose2d(12,-43, Math.toRadians(90.00)))
                .strafeToConstantHeading(InTake);


        telemetry.addData("Status","READDDYYYY ");
        telemetry.update();

        waitForStart();





        if (isStopRequested()) return;

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }

    private void moveArmTo(DcMotor arm,int pos) {
        arm.setTargetPosition(pos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

    }

}
