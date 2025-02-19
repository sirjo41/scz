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
    Servo fingers;
    Servo elbow;
    Servo shoulder;
    Servo wrist;


    public static double FINGERS_OPEN = 0.4;
    public static double FINGERS_CLOSE = 0.6;
    public static double ELBOW_INTAKE = 0.8;
    public static double SHOULDER_INTAKE = 1;
    public static double WRIST_INTAKE = 1;

    public IntakeServos(HardwareMap hardwareMap) {
        // Initialize servos
        fingers = hardwareMap.servo.get("fingers");
        elbow  = hardwareMap.servo.get("elbow");
        shoulder = hardwareMap.servo.get("shoulder");
        wrist = hardwareMap.servo.get("wrist");

//        // Stop all servos initially
        fingers.setPosition(FINGERS_CLOSE);
        elbow.setPosition(ELBOW_INTAKE);
        shoulder.setPosition(SHOULDER_INTAKE);
        wrist.setPosition(WRIST_INTAKE);
    }
    public Action openfingers() {
        return new SetServoPositionAction(fingers, FINGERS_OPEN);
    }

    public Action closefingers() {
        return new SetServoPositionAction(fingers, FINGERS_CLOSE);
    }

    // Action to run the intake forward for 2 seconds
    private static class SetServoPositionAction implements Action {

        private final Servo servo;
        private final double position;

        public SetServoPositionAction(Servo servo, double position) {
            this.servo = servo;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Sets servo to desired position
            servo.setPosition(position);
            packet.put("Servo Position", position);
            // Return false so it completes immediately (one-time set)
            return false;
        }
    }
}
