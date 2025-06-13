package org.firstinspires.ftc;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@TeleOp(name = "Vision OpMode")
public class VisionOpMode extends LinearOpMode {

    private CameraSubsystem cameraSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

        cameraSubsystem = new CameraSubsystem(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Object Detected:", cameraSubsystem.isObjectDetected());
            telemetry.update();
        }

    }
}
