package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServos {
    private final Servo intakeServo1;
    private final Servo intakeServo2;
    private final Servo rs;

    // Constants for intake servo positions
    private static final double INTAKE_OPEN_POSITION = 1.0;
    private static final double INTAKE_CLOSED_POSITION = 0.0;

    // Constants for the additional servo (`rs`)
    private static final double RS_DEFAULT = 0.8;
    private static final double RS_CLOSED = 0.0;
    private static final double RS_OPEN = 1.0;

    public IntakeServos(HardwareMap hardwareMap) {
        // Initialize servos
        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        rs = hardwareMap.get(Servo.class, "rs");

        // Set initial positions
        intakeServo1.setPosition(INTAKE_CLOSED_POSITION);
        intakeServo2.setPosition(INTAKE_OPEN_POSITION);
        rs.setPosition(RS_DEFAULT);
    }

    // Action to open the intake
    public Action openIntake() {
        return new IntakeAction(INTAKE_OPEN_POSITION, INTAKE_CLOSED_POSITION);
    }

    // Action to close the intake
    public Action closeIntake() {
        return new IntakeAction(INTAKE_CLOSED_POSITION, INTAKE_OPEN_POSITION);
    }

    // Action to set the `rs` servo to closed position
    public Action setRsR1() {
        return new RsAction(RS_CLOSED);
    }

    // Action to set the `rs` servo to open position
    public Action setRsL1() {
        return new RsAction(RS_OPEN);
    }

    // Action to reset `rs` to default position
    public Action resetRs() {
        return new RsAction(RS_DEFAULT);
    }

    // Internal action class for controlling the intake servos
    private class IntakeAction implements Action {
        private final double position1;
        private final double position2;

        public IntakeAction(double position1, double position2) {
            this.position1 = position1;
            this.position2 = position2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServo1.setPosition(position1);
            intakeServo2.setPosition(position2);

            // Provide telemetry
            packet.put("Intake Servo 1", position1);
            packet.put("Intake Servo 2", position2);

            return false; // Action completes immediately
        }
    }

    // Internal action class for controlling the `rs` servo
    private class RsAction implements Action {
        private final double position;

        public RsAction(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rs.setPosition(position);

            // Provide telemetry
            packet.put("RS Servo Position", position);

            return false; // Action completes immediately
        }
    }
}
