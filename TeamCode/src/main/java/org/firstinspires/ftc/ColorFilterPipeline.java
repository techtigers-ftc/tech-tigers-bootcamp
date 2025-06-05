package org.firstinspires.ftc;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorFilterPipeline implements VisionProcessor {

    private Scalar lowerBound; // Lower HSV bound
    private Scalar upperBound; // Upper HSV bound
    private boolean objectDetected = false; // Flag to indicate object detection


    public ColorFilterPipeline(Scalar lowerBound, Scalar upperBound) {
        super();
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to HSV color space
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Apply the color filter
        Mat mask = new Mat();
        Core.inRange(hsvFrame, lowerBound, upperBound, mask);

        // Find contours in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Analyze contours and check if any valid objects are detected
        objectDetected = false;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 50) { // Filter out small blobs
                objectDetected = true;
                break;
            }
        }
        return mask; // Return the mask as the processed frame
    }

    public boolean isObjectDetected() {
        return objectDetected; // Return the detection status
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}