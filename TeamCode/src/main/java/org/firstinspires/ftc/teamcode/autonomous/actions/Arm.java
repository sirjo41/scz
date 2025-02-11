package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm {
    private final DcMotor arm;
    private final PIDController armController;

    // PIDF Constants (Tunable in FTC Dashboard)
    public static double p = 0.015, i = 0, d = 0.001;
    public static double f = -0.05;

    // Correct Ticks per Degree for GoBILDA 117 RPM with 5:1 Gear Ratio
    private final double ticks_in_deg = 2688.5 / 360.0;  // ~7.468 ticks per degree

    // Arm Position Stages (Using Ticks)
    public static final int STAGE_0 = 0;
    public static final int STAGE_1 = -50;  // 0 degrees
    public static final int STAGE_2 = 50;   // 90 degrees
    public static final int STAGE_3 = 100;  // 160 degrees

    private double targetPosition = STAGE_0; // Default target

    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Optionally set ZeroPowerBehavior to BRAKE
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PID Controller
        armController = new PIDController(p, i, d);

        // Set motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Continuously apply PIDF to hold targetPosition
    public void maintainPosition() {
        armController.setPID(p, i, d);
        int arm_pos = arm.getCurrentPosition();
        double pid = armController.calculate(arm_pos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / ticks_in_deg)) * f;
        double power = pid + ff;
        arm.setPower(power);
    }

    // Creates a Road Runner action to move the arm to a specific angle
    public Action moveToPositionAction(int targetDegrees) {
        return new ArmPIDFAction(targetDegrees);
    }

    // Creates an Action that runs maintainPosition() indefinitely
    // until we explicitly call stop(). Useful to keep the arm from falling.
    public MaintainArmAction maintainArmForever() {
        return new MaintainArmAction();
    }

    // Actual action class that moves once to a target, then completes
    private class ArmPIDFAction implements Action {
        private final double targetTicks;
        private boolean initialized = false;

        public ArmPIDFAction(double targetDegrees) {
            this.targetTicks = targetDegrees * ticks_in_deg;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // On first iteration, set the new target
            if (!initialized) {
                targetPosition = targetTicks;
                initialized = true;
            }

            // Maintain arm with PID each cycle
            maintainPosition();

            // Send telemetry
            packet.put("Arm Target (Ticks)", targetPosition);
            packet.put("Arm Current Position", arm.getCurrentPosition());

            // Stop action once within ~5 ticks
            return Math.abs(arm.getCurrentPosition() - targetPosition) <= 5;
        }
    }

    // Indefinite arm-holding action
    public class MaintainArmAction implements Action {
        private boolean done = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Always run maintainPosition until told to stop
            maintainPosition();

            // This returns 'true' only if we set done=true (stop() called)
            return done;
        }

        // Call this externally if you want to end the hold
        public void stop() {
            done = true;
        }
    }

    // Helper methods for quick referencing
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
