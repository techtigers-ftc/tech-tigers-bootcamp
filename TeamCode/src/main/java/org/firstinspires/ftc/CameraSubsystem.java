package org.firstinspires.ftc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

public class CameraSubsystem extends SubsystemBase {

    private WebcamName camera;
    private Scalar lowerBound = new Scalar(155, 0, 0); // Lower HSV bound
    private Scalar upperBound = new Scalar(255, 255, 255); // Upper HSV bound
    private Size cameraResolution = new Size(640, 480);
    private final VisionPortal visionPortal;// Camera resolution
    private final ColorFilterPipeline pipeline;
    private boolean objectDetected = false; // Flag to indicate object detection

    public CameraSubsystem(HardwareMap hardwareMap) {
        camera = hardwareMap.get(WebcamName.class, "camera");

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

    public void stop(){
        visionPortal.close();
    }

    public boolean isObjectDetected() {
        return objectDetected;
    }

    @Override
    public void periodic() {
        objectDetected = pipeline.isObjectDetected();
    }
}
