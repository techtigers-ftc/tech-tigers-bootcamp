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
        // This implementation does not require this
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        Core.inRange(frame, lowerBound, upperBound, frame);

        return frame; // Return the mask as the processed frame
    }

    public boolean isObjectDetected() {
        return objectDetected; // Return the detection status
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // This implementation does not require this
    }
}