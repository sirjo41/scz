package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final DcMotor arm;
    private Thread armThread;
    private volatile boolean running = true;

    // PIDF Constants (Tunable in FTC Dashboard)
    public volatile static double p = 0.015, i = 0.5, d = 0.001;
    public volatile  static double f = -0.05;

    // Correct Ticks per Degree for GoBILDA 117 RPM with 5:1 Gear Ratio
    private final double ticks_in_deg = 2688.5 / 360.0;  // ~7.468 ticks per degree

    // Arm Position Stages (Using Ticks)
    public  static final int STAGE_0 = 0;
    public static final int STAGE_1 = -50;  // 0 degrees
    public static final int STAGE_2 = 50;   // 90 degrees
    public static final int STAGE_3 = 100;   // 160 degrees

    private volatile  double targetPosition = STAGE_0; // Default target

    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Initialize PID Controller

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
                startArmThread();
            }

            packet.put("Arm Target (Ticks)", targetPosition);
            packet.put("Arm Current Position", arm.getCurrentPosition());

            return Math.abs(arm.getCurrentPosition() - targetPosition) > 5;
        }
    }

    private void startArmThread() {
        if (armThread == null || !armThread.isAlive()) {
            armThread = new Thread(() -> {
                PIDController armController = new PIDController(p, i, d);
                while (running) {
                    armController.setPID(p, i, d);
                    int arm_pos = arm.getCurrentPosition();
                    double pid = armController.calculate(arm_pos, targetPosition);
                    double ff = Math.cos(Math.toRadians(targetPosition / ticks_in_deg)) * f;
                    double power = pid + ff;
                    arm.setPower(power);

                    try {
                        Thread.sleep(20); // 20ms loop time
                    } catch (InterruptedException e) {
                        return; // end this thread
                    }
                }
            });
            armThread.start();
        }
    }

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

    public void stop() {
        if (armThread != null) {
            try {
                armThread.interrupt(); // make it stop
                armThread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}