package org.firstinspires.commands;

import org.firstinspires.RobotState;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.subsystems.DriveSubsystem;

/**
 * A command that turns the robot to a specified angle.
 * The robot will continue turning until it reaches the target heading within a tolerance.
 */
public class TurnToAngleCommand implements Command {
    private final RobotState robotState;
    private final double targetHeading;
    private final double turnPower;
    private final DriveSubsystem driveSubsystem;

    /**
     * Constructs a TurnToAngleCommand.
     *
     * @param robotState The current state of the robot, used to get its heading.
     * @param driveSubsystem The subsystem responsible for driving the robot.
     * @param turnPower The power to apply for turning.
     * @param targetHeading The target heading in degrees.
     */
    public TurnToAngleCommand(RobotState robotState,
                              DriveSubsystem driveSubsystem,
                              double turnPower,
                              double targetHeading) {
        this.robotState = robotState;
        this.driveSubsystem = driveSubsystem;
        this.turnPower = turnPower;
        this.targetHeading = targetHeading;
    }

    @Override
    public void initialize() {
        driveSubsystem.drive(0, 0, turnPower);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(robotState.getRobotPose().getHeading(AngleUnit.DEGREES) - targetHeading) < 5;
    }

    @Override
    public void end() {
        driveSubsystem.drive(0, 0, 0);
    }
}
