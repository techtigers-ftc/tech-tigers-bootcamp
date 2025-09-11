package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is an OpMode that demonstrates a "shooting" operation where the robot drives for a fixed
 * amount of time and then releases the ball.
 */
@TeleOp(name = "Shooter OpMode", group = "Linear OpMode")
public class ShooterOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystems
        ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(hardwareMap, telemetry);
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        // This is a timer object. We can use it to measure time across
        // loops, and it is useful for timing operations.
        ElapsedTime timer = new ElapsedTime();

        // Any code before this point runs once ONCE when you press INIT
        waitForStart();

        // Any code after this point runs ONCE after you press START
        timer.reset();

        String robotStatus = "READY";

        // Any code in this loop runs REPEATEDLY until the driver presses STOP
        while (opModeIsActive()) {
            if (robotStatus.equals("READY")) {
                // The robot is ready. Check for button presses to start shooting or grip the ball
                if (gamepad1.a) {
                    timer.reset(); // Reset the timer when starting to drive
                    robotStatus = "DRIVING"; // Mark that we are now driving
                } else if(gamepad1.b) {
                    manipulatorSubsystem.sweeperLeft(); // Move the sweeper to the left
                    robotStatus = "GRIPPING"; // Mark that we are now gripping
                } else if (gamepad1.x) {
                    manipulatorSubsystem.sweeperRight(); // Move the sweeper to the right
                    robotStatus = "GRIPPING"; // Mark that we are now gripping
                }
            } else if (robotStatus.equals("DRIVING")) {
                // Check if 0.5 seconds have passed
                if (timer.seconds() > 0.5) {
                    // It has been 0.5 seconds, so stop driving and release the ball
                    driveSubsystem.drive(0, 0, 0); // Stop driving
                    manipulatorSubsystem.resetSweeper(); // Reset the sweeper position
                    timer.reset(); // Reset the timer for the shooting phase
                    robotStatus = "SHOOTING"; // Mark that we are now shooting
                } else {
                    // The robot is currently driving.
                    driveSubsystem.drive(0.5, 0, 0);
                }
            } else if(robotStatus.equals("SHOOTING")) {
                // It typically takes some time for the servo to complete its movement. We use a
                // timer to wait for the servo to finish moving.
                if (timer.seconds() > 0.2) {
                    // It has been enough time since we started shooting, so reset the sweeper
                    robotStatus = "READY"; // Mark that we are now stopped
                }
            } else {
                // It typically takes some time for the servo to complete its movement. We use a
                // timer to wait for the servo to finish moving.
                if (timer.seconds() > 0.2) {
                    // It has been enough time since we started shooting, so reset the sweeper
                    robotStatus = "READY"; // Mark that we are now stopped
                }
            }
            telemetry.addData("Robot Status", robotStatus);
            telemetry.update();
        }
        // Any code after the while loop will run ONCE after the driver presses STOP
    }
}
