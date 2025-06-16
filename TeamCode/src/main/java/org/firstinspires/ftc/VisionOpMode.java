package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vision OpMode")
public class VisionOpMode extends LinearOpMode {

    private CameraSubsystem cameraSubsystem;
    /**
     * This OpMode initializes the CameraSubsystem and continuously checks for object detection.
     * It displays the detection status on the telemetry.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        cameraSubsystem = new CameraSubsystem(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Check Camera Stream for Detection");
            telemetry.update();
        }

    }
}
