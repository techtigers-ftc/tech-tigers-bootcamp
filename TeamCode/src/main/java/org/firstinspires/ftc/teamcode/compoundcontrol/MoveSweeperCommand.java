package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command that will move the sweeper to a specified position
 * This can be used to perform different actions such as releasing, gripping, and passing the ball.
 */
public class MoveSweeperCommand extends Command {
    private final SweeperSubsystem sweeperSubsystem;
    private final String targetPos;
    private final double waitDuration;
    private final ElapsedTime timer;

    /**
     * Constructs the MoveSweeperCommand
     *
     * @param sweeperSubsystem the sweeper subsystem
     * @param targetPos        the servo's target position. This can be one of "LEFT", "RIGHT", or
     *                         "RESET". Any other value will reset the sweeper. Values are not case
     *                         sensitive.
     * @param waitDuration     the time that the command waits for the servo to complete it's movement
     *                         in seconds
     */
    public MoveSweeperCommand(SweeperSubsystem sweeperSubsystem, String targetPos, double waitDuration) {
        this.sweeperSubsystem = sweeperSubsystem;
        // Convert the position to uppercase to protect against users proving mixed case values
        this.targetPos = targetPos.toUpperCase();
        this.waitDuration = waitDuration;
        timer = new ElapsedTime();
    }

    /**
     * Initializes the subsystem, runs once when the command starts
     */
    @Override
    public void initialize() {
        // Any servo action only needs to be triggered once. So we do this in initialize instead of
        // repeatedly invoking it in execute.
        if (targetPos.equals("LEFT")) {
            sweeperSubsystem.sweeperLeft();
        } else if (targetPos.equals("RIGHT")) {
            sweeperSubsystem.sweeperRight();
        } else {
            // Anything that isn't left or right resets the sweeper
            sweeperSubsystem.resetSweeper();
        }
        timer.reset();
    }

    /**
     * Executes the command, runs repeatably while the command is running
     */
    @Override
    public void execute() {
        // This command does not need to do anything in execute since the servo action is triggered
        // in initialize and the command just waits for the action to complete.
    }

    /**
     * Returns a boolean that determines if the command is finished or not
     */
    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.waitDuration;
    }
}
