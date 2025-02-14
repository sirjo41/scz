package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class IntakeServos {
    private final Servo wrist;
    private final CRServo intakeServo1;
    private final CRServo intakeServo2;

    // Constants for intake servo power
    public static final double INTAKE_POWER = 1.0;
    public static final double STOP_POWER = 0.0;

    // Constants for the wrist positions
    public static double WRIST_DEFAULT = 0.5;
    public static double WRIST_OUTTAKE = 0.2;
    public static double WRIST_INTAKE = 0;

    public IntakeServos(HardwareMap hardwareMap) {
        // Initialize servos
        intakeServo1 = hardwareMap.crservo.get("Lin");
        intakeServo2 = hardwareMap.crservo.get("Rin");
        wrist = hardwareMap.get(Servo.class, "Wrist");

        // Stop all servos initially
        intakeServo1.setPower(STOP_POWER);
        intakeServo2.setPower(STOP_POWER);
        wrist.setPosition(WRIST_DEFAULT);
    }

    // Action to run the intake forward for 2 seconds
    public Action Intake() {
        return new IntakeAction(INTAKE_POWER);
    }

    // Action to run the intake in reverse for 2 seconds
    public Action Outtake() {
        return new IntakeAction(-INTAKE_POWER);
    }

    // Action to set wrist to open position
    public Action setWristInTake() {
        return new WristAction(WRIST_INTAKE);
    }

    // Action to set wrist to closed position
    public Action setWristOutTake() {
        return new WristAction(WRIST_OUTTAKE);
    }

    // Action to reset wrist to default position
    public Action setWristOutDf() {
        return new WristAction(WRIST_DEFAULT);
    }


    // Internal action class for controlling the intake servos with a 2-second delay
    private class IntakeAction implements Action {
        private final double power;
        private long startTime = -1;

        public IntakeAction(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == -1) {
                startTime = System.currentTimeMillis();
                intakeServo1.setPower(power);
                intakeServo2.setPower(-power);
            }

            long elapsedTime = System.currentTimeMillis() - startTime;
            if (elapsedTime >= 1000) { // 2 seconds
                intakeServo1.setPower(STOP_POWER);
                intakeServo2.setPower(STOP_POWER);
                return false; // Action completes
            }

            // Provide telemetry
            packet.put("Intake Power", power);
            packet.put("Elapsed Time", elapsedTime / 1000.0);
            return true; // Continue running
        }
    }

    // Internal action class for controlling the wrist with a 2-second delay
    private class WristAction implements Action {
        private final double position;
        public WristAction(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(position);
                return false;
        }
    }
}
