package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Camera", group = "AutonT")
public class AutoOpMode extends LinearOpMode {
    OpenCvCamera camera;
    SampleDetectionPipeline pipeline;
    private OpenCvWebcam webcam;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the dashboard
        dashboard = FtcDashboard.getInstance();

        // Create the pipeline and pass telemetry
        pipeline = new SampleDetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 30);  // Stream at 30 FPS
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error:", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        // The pipeline runs continuously, and the telemetry updates with detected samples
        while (opModeIsActive()) {
            // Keep the OpMode active while streaming
            sleep(100); // Adjust sleep as needed for stability
        }

        // Stop streaming when OpMode ends
        webcam.stopStreaming();
    }
}
