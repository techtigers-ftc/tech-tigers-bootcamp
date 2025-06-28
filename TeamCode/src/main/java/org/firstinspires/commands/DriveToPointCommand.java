package org.firstinspires.commands;

import org.firstinspires.subsystems.DriveSubsystem;
import org.firstinspires.RobotState;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A command that drives the robot to a specified point in the field.
 * The robot will drive forward and strafe until it reaches the target coordinates.
 */
public class DriveToPointCommand implements Command {
    private final RobotState robotState;
    private final double targetX;
    private final double targetY;
    private final double forwardPower;
    private final double strafePower;
    private final DriveSubsystem driveSubsystem;

    /**
     * Constructs a DriveToPointCommand.
     *
     * @param robotState The current state of the robot, used to get its position.
     * @param driveSubsystem The subsystem responsible for driving the robot.
     * @param forwardPower The power to apply for forward movement.
     * @param strafePower The power to apply for strafing movement.
     * @param targetX The target X coordinate in centimeters.
     * @param targetY The target Y coordinate in centimeters.
     */
    public DriveToPointCommand(RobotState robotState,
                               DriveSubsystem driveSubsystem,
                               double forwardPower,
                               double strafePower, double targetX,
                               double targetY) {
        this.robotState = robotState;
        this.driveSubsystem = driveSubsystem;
        this.forwardPower = forwardPower;
        this.strafePower = strafePower;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    @Override
    public void initialize() {
        driveSubsystem.drive(forwardPower, strafePower, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(robotState.getRobotPose().getX(DistanceUnit.CM) - targetX) < 5 &&
               Math.abs(robotState.getRobotPose().getY(DistanceUnit.CM) - targetY) < 5;
    }

    @Override
    public void end() {
        driveSubsystem.drive(0, 0, 0);
    }
}
