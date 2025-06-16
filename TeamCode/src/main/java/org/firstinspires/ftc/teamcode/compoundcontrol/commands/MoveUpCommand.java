package org.firstinspires.ftc.teamcode.compoundcontrol.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.compoundcontrol.subsystems.ManipulatorSubsystem;

/**
 * Command to move the manipulator arm up, close the claw, and pitch the claw
 * to a drop position.
 */
public class MoveUpCommand {
    private final ManipulatorSubsystem manipulatorSubsystem;
    private boolean isFinished;
    private ElapsedTime timer;
    private boolean movingClaw;
    private boolean movingClawPitch;
    private boolean movingArm;
    private final Telemetry telemetry;

    /**
     * Constructor for the MoveUpCommand.
     *
     * @param subsystem The manipulator subsystem to control.
     * @param opmodeTelemetry The telemetry object for logging.
     */
    public MoveUpCommand(ManipulatorSubsystem subsystem, Telemetry opmodeTelemetry) {
        manipulatorSubsystem = subsystem;
        telemetry = opmodeTelemetry;
        isFinished = true;
        timer = new ElapsedTime();

        movingClaw = false;
        movingClawPitch = false;
        movingArm = false;
    }

    /**
     * Initializes the command by closing the claw and resetting the state.
     */
    public void initialize() {
        manipulatorSubsystem.closeClaw();
        movingClaw = true;
        movingClawPitch = false;
        movingArm = false;

        isFinished = false;
        timer.reset();
    }

    /**
     * Executes the command to move the manipulator arm up, close the claw,
     * and pitch the claw to a drop position.
     */
    public void execute() {
        if (movingClaw) { // If the claw is moving
            if (timer.seconds() > 0.5) { // Wait for claw to close
                movingClaw = false;
                movingClawPitch = true;
                manipulatorSubsystem.pitchClawToDrop();
                timer.reset();
            }
        } else if (movingClawPitch) { // If the claw pitch is moving
            if (timer.seconds() > 0.5) {
                movingClawPitch = false;
                movingArm = true;
                manipulatorSubsystem.moveArmToDrop();
                timer.reset();
            }
        } else { // If the arm is moving
            if (!manipulatorSubsystem.isArmBusy()) { // Wait for arm to finish
                movingArm = false;
                isFinished = true; // Reset for next operation
            }
        }

        telemetry.addLine();
        telemetry.addData("Claw moving", movingClaw);
        telemetry.addData("Claw Pitch moving", movingClawPitch);
        telemetry.addData("Arm moving", movingArm);
    }

    /**
     * Checks if the command is finished.
     *
     * @return true if the command is finished, false otherwise.
     */
    public boolean isFinished() {
        return isFinished;
    }
}
