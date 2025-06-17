package org.firstinspires.ftc;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

/**
 * CameraSubsystem is a class that manages the camera for detecting objects
 * using OpenCV color filtering. It initializes the camera, sets up the color filter pipeline,
 */
public class CameraSubsystem {

    private Scalar lowerBound = new Scalar(90, 150, 20); // Lower HSV bound
    private Scalar upperBound = new Scalar(140, 255, 255); // Upper HSV bound
    private Size cameraResolution = new Size(640, 480);
    private final VisionPortal visionPortal;// Camera resolution
    private final ColorFilterPipeline pipeline;

    /**
     * Constructor for CameraSubsystem.
     * @param hardwareMap The hardware map to access the robot's hardware
     */
    public CameraSubsystem(HardwareMap hardwareMap) {

        pipeline = new ColorFilterPipeline(
                lowerBound, // Lower HSV bound
                upperBound // Upper HSV bound
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(pipeline)
                .setCameraResolution(cameraResolution)
                .build();
    }

    /**
     * Stops the camera subsystem by closing the vision portal.
     */
    public void stop(){
        visionPortal.close();
    }
}
