package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    // Define color ranges in HSV with extended saturation and brightness ranges
    private final Scalar yellowLower = new Scalar(20, 100, 100);
    private final Scalar yellowUpper = new Scalar(30, 255, 255);
    private final Scalar redLower = new Scalar(0, 100, 100);
    private final Scalar redUpper = new Scalar(10, 255, 255);
    private final Scalar UredLower = new Scalar(160, 100, 100);
    private final Scalar UredUpper = new Scalar(179, 255, 255);
    private final Scalar blueLower = new Scalar(78, 50, 50);
    private final Scalar blueUpper = new Scalar(140, 255, 255);

    // Reusable Mats for processing
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private String detectedColor = "None";
    private String direction = "None";
    private final int minArea = 500; // Minimum area for large color chunks
    private int frameCenterX;
    private int frameCenterY;
    public int sampleCenterXn;
    public int sampleCenterYn;
    public double size;

    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Calculate frame center
        frameCenterX = input.width() / 2;
        frameCenterY = input.height() / 2;

        // Reset detected color and direction for each frame
        detectedColor = "None";
        direction = "None";

        // Check for yellow, red (across both ranges), and blue chunks in that order
        if ( !detectColor(hsvMat, input, redLower, redUpper, "Red" ) &&!detectColor(hsvMat,input,UredLower,UredUpper,"RED") ) {
            if (!detectColor(hsvMat, input, blueLower, blueUpper, "Blue")) {
                detectColor(hsvMat, input, yellowLower, yellowUpper, "Yellow");
            }
        }

        // Update telemetry with the detected color and direction
        updateTelemetry();

        return input; // Return the original frame for visualization
    }

    private boolean detectColor(Mat hsvMat, Mat input, Scalar lower, Scalar upper, String color) {
        Core.inRange(hsvMat, lower, upper, mask);

        // Find contours to check for detected colors
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Only consider contours above the minimum area
            if (Imgproc.contourArea(contour) > minArea) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                size = Imgproc.contourArea(contour);
                // Draw the bounding box around the detected color
                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 2);

                // Calculate the centroid of the bounding box
                int sampleCenterX = boundingRect.x + (boundingRect.width / 2);
                int sampleCenterY = boundingRect.y + (boundingRect.height / 2);

                sampleCenterXn = boundingRect.x + (boundingRect.width / 2);
                sampleCenterYn = boundingRect.y + (boundingRect.height / 2);
                // Determine the direction based on the position of the bounding box
                direction = getDirection(sampleCenterX, sampleCenterY);

                // Set the detected color and stop processing additional contours
                detectedColor = color;
                return true; // Return true to stop further color detection
            }
        }
        return false; // No color detected, continue with the next color
    }

    private String getDirection(int x, int y) {
        // Determine the general direction based on the object's position relative to the frame center
        if (y < frameCenterY ) {
            return "Forward";
        }else if(y > frameCenterY){
            return "Back";
        }
        else if (x < frameCenterX) {
            return "Left";
        } else if (x > frameCenterX) {
            return "Right";
        } else {
            return "Done";
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Color Heading To:", detectedColor);
        telemetry.addData("Direction:", direction);
        telemetry.addData("Frame Center X:", sampleCenterXn);
        telemetry.addData("Frame Center Y:", sampleCenterYn);
        telemetry.addData("Size:", size);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Color Heading To", detectedColor);
        packet.put("Direction", direction);
        packet.put("Frame Center X", sampleCenterXn);
        packet.put("Frame Center Y", sampleCenterYn);
        packet.put("Size:",size);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void onViewportTapped() {
        // Release Mats on pipeline closure to free memory
        hsvMat.release();
        mask.release();
    }
}
