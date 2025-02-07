package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NonNls;

public class Slides {
    private DcMotor slide1;
    private DcMotor slide2;

    // Constants for stage positions
    public static final int STAGE_0 = 5;
    public static final int STAGE_1 = 900;
    public static final int STAGE_2 = 1800;
    public static final int STAGE_3 = 2700;

    public Slides(HardwareMap hardwareMap) {
        // Initialize motors
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");

        // Set motor directions
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            if (!initialized) {
                slide2.setTargetPosition(targetPosition);
                slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide2.setPower(1);
                slide1.setPower((targetPosition > slide2.getCurrentPosition() ? 1 : -1) * 0.7);
                initialized = true;
            }

            // Provide telemetry
            packet.put("Slide Target", targetPosition);
            packet.put("Slide Current Position", slide2.getCurrentPosition());

            if (!slide2.isBusy()) {
                slide1.setPower(0);
                slide2.setPower(0);
                return false; // Action is complete
            }
            return true; // Action is still running
        }
    }

    // Predefined stage actions
    public Action goToStage0() {
        return moveToPositionAction(STAGE_0);
    }

    public Action goToStage1() {
        return moveToPositionAction(STAGE_1);
    }

    public Action goToStage2() {
        return moveToPositionAction(STAGE_2);
    }

    public Action goToStage3() {
        return moveToPositionAction(STAGE_3);
    }
}
