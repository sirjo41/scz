package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTelemetry", group = "TeleOp")
public class ArmTelemetry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        // Reset encoder and set mode
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}