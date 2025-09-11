package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for controlling a manipulator with a claw and an arm.
 */
public class ManipulatorSubsystem {
    public final Servo sweeper;

    // These are constant that control the left and right positions of the servo arm. Change as
    // necessary.
    private final double LEFT_SERVO_POSITION = 0.3;
    private final double RIGHT_SERVO_POSITION = 0.75;

    /**
     * Constructor for the ManipulatorSubsystem.
     *
     * @param hardwareMap The hardware map to access the robot's hardware components.
     */
    public ManipulatorSubsystem(HardwareMap hardwareMap) {
        sweeper = hardwareMap.servo.get("sweeper_servo");
    }

    /**
     * Moves sweeper to the left.
     */
    public void sweeperLeft() {
        sweeper.setPosition(LEFT_SERVO_POSITION);
    }

    /**
     * Moves sweeper to the right.
     */
    public void sweeperRight() {
        sweeper.setPosition(RIGHT_SERVO_POSITION);
    }

    /**
     * Resets the sweeper to the neutral position.
     */
    public void resetSweeper() {
        sweeper.setPosition(0.5);
    }
}