package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * This Command will drive the robot forward for a certain amount of time and then stop
 */
public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double duration;
    private final double power;
    private final ElapsedTime timer;

    /**
     * Constructs the Drive command
     * @param driveSubsystem the drive subsystem
     * @param duration the duration for which the command runs in seconds
     * @param power the power at which the robot drives in a range of -1 to +1
     */
    public DriveCommand(DriveSubsystem driveSubsystem, double duration, double power){
        this.driveSubsystem = driveSubsystem;
        this.duration = duration;
        this.power = power;

        timer = new ElapsedTime();
    }

    /**
     * Initializes the subsystem, runs once when the command starts
     */
    @Override
    public void initialize(){
        timer.reset();
    }

    /**
     * Executes the command, this method will be called repeatedly until the command finishes
     */
    @Override
    public void execute(){
        driveSubsystem.drive(this.power, 0, 0);
        if (this.isFinished()){
            // The robot has driven for the specified time so the robot should stop now
            driveSubsystem.drive(0, 0,0 );
        } else {
            // Time has not yet elapsed, keep on driving
            driveSubsystem.drive(this.power, 0, 0);
        }
    }

    /**
     * Returns a boolean that determines if the command is finished or not
     */
    @Override
    public boolean isFinished(){
        return timer.seconds() >= this.duration;
    }
}