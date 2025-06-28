package org.firstinspires.commands;

/**
 * This interface defines the structure for a command in a command-based robot framework.
 * Each command must implement the methods to initialize, execute, and check if it is finished.
 */
public interface Command {

    /**
     * Initializes the command. This method is called once when the command is scheduled.
     */
    void initialize();

    /**
     * Executes the command. This method is called repeatedly while the command is scheduled.
     */
    void execute();

    /**
     * Checks if the command has finished executing.
     *
     * @return true if the command is finished, false otherwise.
     */
    boolean isFinished();

    /**
     * Cleans up after the command has finished executing.
     * This method is called once when the command ends.
     */
    void end();
}
