package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "LocalisationTestOpMode")
public class LocalisationTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();

        Pose2D startPose = new Pose2D(DistanceUnit.MM, 0, 0,
                AngleUnit.RADIANS, 0);
        // Initialize the odometry subsystem
        OdometrySubsystem odometrySubsystem =
                new OdometrySubsystem(hardwareMap, robotState, startPose);

        // Wait for the start signal
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update the odometry
            odometrySubsystem.periodic();

            // Display the current pose on telemetry
            telemetry.addData("Current Pose", robotState.getCurrentPose());
            telemetry.addData("Current Velocity", robotState.getCurrentVelocity());

            telemetry.update();
        }
    }
}
