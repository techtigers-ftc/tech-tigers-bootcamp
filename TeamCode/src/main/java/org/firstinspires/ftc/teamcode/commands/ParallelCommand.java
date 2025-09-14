package org.firstinspires.ftc.teamcode.commands;

/**
 * Abstract base class sequential commands. A sequential command maintains a list of commands and
 * runs them sequentially.
 */
public class ParallelCommand extends Command {
    private final Command[] commands;

    /**
     * Constructs the SequentialCommand
     * The assumption is that there is at least one command in the command list. Real production
     * code should validate this.
     *
     * @param commands the list of commands to run sequentially
     */
    public ParallelCommand(Command[] commands) {
        this.commands = commands;
        // Both of these will be updated in the initialize method
    }

    /**
     * Initializes the command, runs once when the command starts
     */
    @Override
    public void initialize() {
        for (Command command : commands) {
            command.initialize();
        }
    }

    /**
     * Executes the command, runs repeatably while the command is running
     */
    @Override
    public void execute() {
        if (!isFinished()) {
            for (Command command : commands) {
                if (!command.isFinished()) {
                    command.execute();
                }
            }
        }
    }

    /**
     * Returns a boolean that determines if the command is finished or not
     *
     * @return true if all commands have been executed, false otherwise
     */
    @Override
    public boolean isFinished() {
        int commandFinishedCounter = 0;
        for (Command command : commands) {
            if (command.isFinished()) {
                commandFinishedCounter++;
            }
        }
        return commandFinishedCounter >= commands.length;
    }
}
