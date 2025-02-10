package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm {
    private final DcMotor arm;
    private final PIDController armController;
    private Thread holdThread;
    private boolean isHolding = false;  // Track whether the arm should keep holding

    // PIDF Constants (Tunable in FTC Dashboard)
    public static double p = 0.01, i = 0, d = 0.002;
    public static double f = 0.05;

    // Correct Ticks per Degree for GoBILDA 117 RPM with 5:1 Gear Ratio
    private final double ticks_in_deg = 2688.5 / 360.0;  // ~7.468 ticks per degree

    // Arm Position Stages (Using Ticks)
    public static final int STAGE_0 = (int) (0 * 7.468);      // 0 degrees
    public static final int STAGE_1 = (int) (90 * 7.468);     // 90 degrees
    public static final int STAGE_2 = (int) (160 * 7.468);    // 160 degrees

    private double targetPosition = STAGE_0; // Default target

    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Initialize PID Controller
        armController = new PIDController(p, i, d);

        // Set motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Initialize encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action moveToPositionAction(int targetDegrees) {
        return new ArmPIDFAction(targetDegrees);
    }

    private class ArmPIDFAction implements Action {
        private final double targetTicks;
        private boolean initialized = false;

        public ArmPIDFAction(double targetDegrees) {
            this.targetTicks = targetDegrees * ticks_in_deg;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                targetPosition = targetTicks;
                initialized = true;
            }

            armController.setPID(p, i, d);
            int arm_pos = arm.getCurrentPosition();
            double pid = armController.calculate(arm_pos, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticks_in_deg)) * f;
            double power = pid + ff;

            arm.setPower(power);

            packet.put("Arm Target (Ticks)", targetPosition);
            packet.put("Arm Current Position", arm_pos);
            packet.put("Power", power);

            // **Start Holding the Position in a Background Thread**
            startHoldingPosition();

            return Math.abs(arm_pos - targetPosition) > 10;  // Keep running if not close enough
        }
    }

    // **Predefined Stages with Holding**
    public Action goToStage0() {
        return new SequentialAction(
                moveToPositionAction(STAGE_0),
                holdPositionAction()
        );
    }

    public Action goToStage1() {
        return new SequentialAction(
                moveToPositionAction(STAGE_1),
                holdPositionAction()
        );
    }

    public Action goToStage2() {
        return new SequentialAction(
                moveToPositionAction(STAGE_2),
                holdPositionAction()
        );
    }

    // **Continuously Holds Position in a Separate Thread**
    public void startHoldingPosition() {
        if (isHolding) return;  // Avoid multiple threads

        isHolding = true;
        holdThread = new Thread(() -> {
            while (isHolding) {
                armController.setPID(p, i, d);
                int arm_pos = arm.getCurrentPosition();
                double pid = armController.calculate(arm_pos, targetPosition);
                double ff = Math.cos(Math.toRadians(targetPosition / ticks_in_deg)) * f;
                double power = pid + ff;
                arm.setPower(power);

                try {
                    Thread.sleep(20);  // Run at ~50Hz
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        holdThread.start();
    }

    // **Stops Holding the Position**
    public void stopHoldingPosition() {
        isHolding = false;
        if (holdThread != null) {
            holdThread.interrupt();
        }
        arm.setPower(0);
    }

    // **Creates a Holding Action That Runs Indefinitely**
    public Action holdPositionAction() {
        return packet -> {
            startHoldingPosition();
            return true;  // Always return true to keep it running
        };
    }
}
