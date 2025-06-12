package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class defines a subsystem for driving a robot with a Mecanum drive system.
 * It handles the initialization of motors and provides a method to control the robot's movement.
 */
public class DriveSubsystem {
    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;
    private final Telemetry telemetry;

    /**
     * Constructor for the DriveSubsystem.
     * Initializes the motors and sets their directions based on the robot's configuration.
     *
     * @param hardwareMap The hardware map to access the robot's hardware
     * @param opModeTelemetry The telemetry object for sending data to the driver station.
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        // Initialize the hardware
        // The strings here must correspond to the names of the motors in your robot config
        leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        leftBackDrive = hardwareMap.dcMotor.get("left_back");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        rightBackDrive = hardwareMap.dcMotor.get("right_back");

        // Set motor directions
        // You may need to adjust these for your particular robot
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry = opModeTelemetry;
    }

    /**
     * Drives the robot using the Mecanum drive system.
     * Combines axial, lateral, and yaw inputs to calculate the power for each wheel.
     *
     * @param axial   The forward/backward movement input (negative for forward)
     * @param lateral The left/right strafe input
     * @param yaw     The rotation input (positive for clockwise rotation)
     */
    public void drive(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
}
