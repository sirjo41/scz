package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "fingers")
public class pos extends LinearOpMode {

    public static double WRIST_INTAKE = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo wrist = hardwareMap.servo.get("wrist");
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            wrist.setPosition(WRIST_INTAKE);
            telemetry.addData("wrist",WRIST_INTAKE);
            telemetry.update();
        }
    }
}
