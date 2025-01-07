package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "slide Manual", group = "TeleOp")
public class teletest extends LinearOpMode {
    private FtcDashboard dashboard;
    private DcMotor slide1;
    private DcMotor slide2;
    @Override
    public void runOpMode() {

        // Initialize the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide2.setDirection(DcMotor.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        // The pipeline runs continuously, and the telemetry updates with detected samples
        while (opModeIsActive()) {

            if(gamepad1.a){
                slide2.setPower(0.5);

            }
            else if (gamepad1.b) {

                slide2.setPower(-0.5);

        }
        else{

                slide2.setPower(0);

            }
            if(gamepad1.a){
                slide1.setPower(0.1);
            }
            else if (gamepad1.b) {
                slide1.setPower(-0.1);
            }
            else{
                slide1.setPower(0);
            }

            telemetry.addData("Slide1",slide1.getCurrentPosition());
            telemetry.addData("Slide1 Power", slide1.getPower());
            telemetry.addData("Slide2 Power", slide2.getPower());
            telemetry.update();


        }
    }
}
