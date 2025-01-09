package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    private DcMotor slide1;
    private DcMotor slide2;

    // Constants for stage positions
    private static final int STAGE_1 = 900;
    private static final int STAGE_2 = 1800;
    private static final int STAGE_3 = 2700;

    public Slides(HardwareMap hardwareMap) {
        // Initialize motors
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");

        // Set motor directions
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);

        // Initialize encoders
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goToStage1() {
        moveToPosition(STAGE_1);
    }

    public void goToStage2() {
        moveToPosition(STAGE_2);
    }

    public void goToStage3() {
        moveToPosition(STAGE_3);
    }

    private void moveToPosition(int targetPosition) {
        int currentPosition = slide2.getCurrentPosition();
        int direction = (targetPosition > currentPosition) ? 1 : -1; // Determine direction

        slide2.setTargetPosition(targetPosition);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setPower(1); // Adjust power
        slide1.setPower(direction * 0.7);

        while (slide2.isBusy()) {

        }

        // Stop motors after reaching the target
        slide1.setPower(0);
        slide2.setPower(0);
    }
}
