package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ManipulatorSubsystem;

/**
 * Command that implements a "shoot" action by driving forward and moving the sweeper to "RESET".
 */
public class ShootCommand extends SequentialCommand {
    /**
     * Constructs the Shoot command. Accepts a list of subsystem to be used by the command.
     *
     * @param manipulatorSubsystem the sweeper subsystem
     * @param driveSubsystem   the drive subsystem
     */
    public ShootCommand(ManipulatorSubsystem manipulatorSubsystem, DriveSubsystem driveSubsystem) {
        // The sequential command implementation for running these commands one after the other
        // exists in the parent class. All we need to do is provide the list of commands to run.
        super(
                new Command[]{
                        new DriveCommand(driveSubsystem, 0.5, 0.5),
                        new MoveSweeperCommand(manipulatorSubsystem, "RESET", 0.2)
                }
        );
    }
}
