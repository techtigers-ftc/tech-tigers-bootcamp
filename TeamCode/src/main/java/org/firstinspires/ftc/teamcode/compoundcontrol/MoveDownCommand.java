package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Command to move the manipulator arm down, open the claw, and pitch the claw
 * to an intake position.
 */
public class MoveDownCommand {
    private final ManipulatorSubsystem manipulatorSubsystem;
    private boolean isFinished;
    private ElapsedTime timer;
    private boolean movingClaw;
    private boolean movingClawPitch;
    private boolean movingArm;
    private final Telemetry telemetry;

    /**
     * Constructor for the MoveDOwnCommand.
     *
     * @param subsystem The manipulator subsystem to control.
     * @param opmodeTelemetry The telemetry object for logging.
     */
    public MoveDownCommand(ManipulatorSubsystem subsystem, Telemetry opmodeTelemetry) {
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
        manipulatorSubsystem.openClaw();
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
            if (timer.seconds() > 0.5) { // Wait for claw to finish
                movingArm = true;
                movingClawPitch = true;
                movingClaw = false;
                manipulatorSubsystem.pitchClawToIntake();
                manipulatorSubsystem.moveArmToIntake();
                timer.reset();
            }
        } else { // If the arm and claw pitch are moving
            if (!manipulatorSubsystem.isArmBusy()) { // Wait for arm and claw pitch to finish moving
                movingArm = false;
                movingClawPitch = false;
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
