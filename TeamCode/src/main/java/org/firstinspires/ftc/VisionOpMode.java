package org.firstinspires.ftc;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@TeleOp(name = "Vision OpMode")
public class VisionOpMode extends LinearOpMode {
    private Scalar lowerBound = new Scalar(155, 0, 0); // Lower HSV bound
    private Scalar upperBound = new Scalar(255, 0, 0); // Upper HSV bound
    private Size cameraResolution = new Size(640, 480); // Camera resolution

    @Override
    public void runOpMode() throws InterruptedException {
        ColorFilterPipeline pipeline = new ColorFilterPipeline(
                lowerBound, // Lower HSV bound
                upperBound // Upper HSV bound
        );
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(pipeline)
                .setCameraResolution(cameraResolution)
                .build();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addLine("Processor Running");
            if (pipeline.isObjectDetected()) {
                telemetry.addLine("Object Detected!");
            } else {
                telemetry.addLine("No Object Detected");
            }
        }

    }
}
