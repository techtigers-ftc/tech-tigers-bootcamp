package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LocalisationTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the odometry subsystem
        OdometrySubsystem odometrySubsystem = new OdometrySubsystem(hardwareMap, new Pose2D(DistanceUnit.MM,0, 0,
                AngleUnit.RADIANS, 0));

        // Wait for the start signal
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update the odometry
            odometrySubsystem.periodic();

            // Get the current pose
            Pose2D currentPose = odometrySubsystem.getCurrentPose();
            Pose2D currentVelocity = odometrySubsystem.getCurrentPose();


            // Display the current pose on telemetry
            telemetry.addData("Current Pose", currentPose);
            telemetry.addData("Current Velocity", currentVelocity);

            telemetry.update();
        }
    }
}
