package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Encoder Test", group = "TeleOp")
public class Encoder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slides1 = hardwareMap.dcMotor.get("slide1");

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        waitForStart();
        slides1.setTargetPosition(1000);
        slides1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides1.setPower(1);

        do {
            telemetry.addData("Target", 1000);
            telemetry.addData("Current Position Slide1", slides1.getCurrentPosition());
            telemetry.update();
        } while (opModeIsActive() && slides1.isBusy());
        slides1.setPower(0);
    }
}
