package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Slides {
    private final DcMotor slide1;
    private final DcMotor slide2;

    // Constants for stage positions
    public static int STAGE_INTAKE = 5;
    public static int STAGE_OUTTAKE = 2000;
    public static int STAGE_OUTTAKE2 = 2300;
    public static int STAGE_3 = 2000;

    public Slides(HardwareMap hardwareMap) {
        // Initialize motors
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        // Set motor directions
        slide1.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Moves to a specific position
    public Action moveToPositionAction(int targetPosition) {
        return new SlideAction(targetPosition);
    }

    private class SlideAction implements Action {
        private final int targetPosition;
        private boolean initialized = false;

        public SlideAction(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            slide1.setTargetPosition(targetPosition);
            slide2.setTargetPosition(targetPosition);
            slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide1.setPower(1);
            slide2.setPower(1);

            if(!slide1.isBusy() && slide2.isBusy()){
            slide1.setPower(0);
            slide2.setPower(0);
            return false;
        }
            return true; // Action is still running
        }
    }

    // Predefined stage actions
    public Action goToStage0() {
        return moveToPositionAction(STAGE_INTAKE);
    }

    public Action goToStage1() {
        return moveToPositionAction(STAGE_OUTTAKE);
    }

    public Action goToStage2() {
        return moveToPositionAction(STAGE_OUTTAKE2);
    }

    public Action goToStage3() {
        return moveToPositionAction(STAGE_3);
    }
}