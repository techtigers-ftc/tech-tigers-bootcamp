package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * This Command will drive the robot forward for a certain amount of time and then stop
 */
public class DriveUntilColorCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final RobotState robotState;
    private final double power;
    private final double threshold;

    /**
     * Constructs the Drive command
     *
     * @param driveSubsystem the drive subsystem
     * @param power          the power at which the robot drives in a range of -1 to +1
     */
    public DriveUntilColorCommand(DriveSubsystem driveSubsystem, RobotState robotState,
                                  double power, double threshold) {
        this.driveSubsystem = driveSubsystem;
        this.power = power;
        this.robotState = robotState;
        this.threshold = threshold;
    }

    @Override
    public void initialize() {

    }

    /**
     * Executes the command, this method will be called repeatedly until the command finishes
     */
    @Override
    public void execute() {
        driveSubsystem.drive(this.power, 0, 0);
        if (this.isFinished()) {
            // The robot has driven for the specified time so the robot should stop now
            driveSubsystem.drive(0, 0, 0);
        } else {
            // Time has not yet elapsed, keep on driving
            driveSubsystem.drive(this.power, 0, 0);
        }
    }

    /**
     * Returns a boolean that determines if the command is finished or not
     */
    @Override
    public boolean isFinished() {
        if (robotState.isDetectRed()) {
            if (robotState.getRed() > threshold) {
                robotState.setDetectRed(false);
                return true;
            }
        } else {
            if (robotState.getBlue() > threshold) {
                robotState.setDetectRed(true);
                return true;
            }
        }
        return false;
    }
}