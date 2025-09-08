package org.firstinspires.ftc.teamcode.commands;

/**
 * Abstract base class for all commands in the command-based framework.
 * Has the basic structure for a command with initialize, execute, and isFinished methods.
 */
public abstract class Command {
    /**
     * Initializes the command, runs once when the command starts
     */
    public abstract void initialize();

    /**
     * Executes the command, runs repeatedly while the command is running
     */
    public abstract void execute();

    /**
     * Returns a boolean that determines if the command is finished or not
     */
    public abstract boolean isFinished();
}
