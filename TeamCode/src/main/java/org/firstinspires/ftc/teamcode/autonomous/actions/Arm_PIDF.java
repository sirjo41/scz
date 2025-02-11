package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class Arm_PIDF extends OpMode {
    private PIDController armController;

    public static double p = 0.015, i = 0, d = 0.001;
    public static double f = -0.05;
    public static double target = 0;

    private final double ticks_in_deg = 2688.5 / 360.0;
    private DcMotorEx arm;

    @Override
    public void init() {
        armController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop() {
        armController.setPID(p, i, d);
        int arm_pos = arm.getCurrentPosition();
        double pid = armController.calculate(arm_pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_deg)) * f;
        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("pos", arm_pos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("f", f);
        telemetry.update();
    }
}