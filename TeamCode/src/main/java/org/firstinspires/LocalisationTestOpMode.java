package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * This OpMode is used to test the localisation subsystem of the robot.
 */
@TeleOp(name = "LocalisationTestOpMode")
public class LocalisationTestOpMode extends LinearOpMode {
    private RobotState robotState;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the odometry subsystem
        robotState = new RobotState();
        OdometrySubsystem odometrySubsystem = new OdometrySubsystem(hardwareMap, robotState,
                new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS, 0));

        // Wait for the start signal
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update the odometry
            odometrySubsystem.periodic();

            // Get the current pose
            Pose2D currentPose = robotState.getRobotPose();
            Pose2D currentVelocity = robotState.getRobotVelocity();


            // Display the current pose on telemetry
            telemetry.addData("Current X", currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("Current Y", currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Current Heading", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("X Velocity", currentVelocity.getX(DistanceUnit.INCH));
            telemetry.addData("Y Velocity", currentVelocity.getY(DistanceUnit.INCH));
            telemetry.addData("Heading Velocity", currentVelocity.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}
