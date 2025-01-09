package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final DcMotor arm;

    // Constants for stage positions
    public static final int STAGE_0 = 0;
    public static final int STAGE_1 = 550;
    public static final int STAGE_2 = 1600;

    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Moves to a specific position
    public Action moveToPositionAction(int targetPosition) {
        return new ArmAction(targetPosition);
    }

    private class ArmAction implements Action {
        private final int targetPosition;
        private boolean initialized = false;

        public ArmAction(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm.setTargetPosition(targetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                initialized = true;
            }

            // Provide telemetry
            packet.put("Arm Target", targetPosition);
            packet.put("Arm Current Position", arm.getCurrentPosition());

            if (!arm.isBusy()) {
                arm.setPower(0.1); // Hold position with minimal power
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
}
