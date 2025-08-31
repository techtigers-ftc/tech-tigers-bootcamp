package org.firstinspires.ftc.teamcode.compoundcontrol;

/**
 * Abstract base class sequential commands. A sequential command maintains a list of commands and
 * runs them sequentially.
 */
public abstract class SequentialCommand extends Command {
    private final Command[] commands;
    private int commandIndex; // An index to track the current command being executed

    /**
     * Constructs the SequentialCommand
     * The assumption is that there is at least one command in the command list. Real production
     * code should validate this.
     *
     * @param commands the list of commands to run sequentially
     */
    protected SequentialCommand(Command[] commands) {
        this.commands = commands;
        // Both of these will be updated in the initialize method
        commandIndex = -1;
    }

    /**
     * Initializes the command, runs once when the command starts
     */
    @Override
    public void initialize() {
        commandIndex = 0; // Start with the first command
        // NOTE: This will error with a index out of bounds if the commands array is empty
        commands[commandIndex].initialize(); // Initialize the first command
    }

    /**
     * Executes the command, runs repeatably while the command is running
     */
    @Override
    public void execute() {
        if (!isFinished()) {
            Command currentCommand = commands[commandIndex];
            // If the command is just starting, initialize it
            if (currentCommand.isFinished()) {
                // Move to the next command
                commandIndex++;
                if (commandIndex < commands.length) {
                    commands[commandIndex].initialize();
                }
            } else {
                // Execute the current command
                currentCommand.execute();
            }
        }
    }

    /**
     * Returns a boolean that determines if the command is finished or not
     * @return true if all commands have been executed, false otherwise
     */
    @Override
    public boolean isFinished() {
        return commandIndex >= commands.length;
    }
}
