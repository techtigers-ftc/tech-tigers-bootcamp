package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name="Compound Control OpMode", group="Linear OpMode")
public class CompoundControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(hardwareMap);

        // This is a timer object. We can use it to measure time across
        // loops, and it is useful for timing operations.
        ElapsedTime timer = new ElapsedTime();

        boolean startedUp = false;
        boolean startedDown = false;
        boolean movingClaw = false;
        boolean movingClawPitch = false;
        boolean movingArm = false;

        waitForStart();

        manipulatorSubsystem.openClaw();
        manipulatorSubsystem.pitchClawToIntake();
        timer.reset();

        while (opModeIsActive()) {
            // This code is going to allow for 2 sequences of operations:
            // 1. Close claw, pitch claw to drop position, move arm up
            // 2. Move arm down, pitch claw to intake position and open claw
            // The sequences are triggered by pressing the dpad up or down.

            if (startedUp) { // If the sequence to move arm up has started
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
                        manipulatorSubsystem.moveArmToPosition(1000, 0.75);
                        timer.reset();
                    }
                } else { // If the arm is moving
                    if (!manipulatorSubsystem.isArmBusy()) { // Wait for arm to finish
                        movingArm = false;
                        startedUp = false; // Reset for next operation
                    }
                }
            } else if (startedDown) { // If the sequence to move arm down has started
                if (movingClaw) { // If the claw is moving
                    if (timer.seconds() > 0.5) { // Wait for claw to finish
                        movingArm = true;
                        movingClawPitch = true;
                        movingClaw = false;
                        manipulatorSubsystem.pitchClawToIntake();
                        manipulatorSubsystem.moveArmToPosition(0, -0.5);
                        timer.reset();
                    }
                } else { // If the arm and claw pitch are moving
                    if (!manipulatorSubsystem.isArmBusy()) { // Wait for arm and claw pitch to finish moving
                        movingArm = false;
                        movingClawPitch = false;
                        startedDown = false; // Reset for next operation
                    }
                }
            } else {
                if (gamepad1.dpad_up) { // Start the sequence to move arm up
                    startedUp = true;
                    movingClaw = true;
                    manipulatorSubsystem.closeClaw();
                    timer.reset();
                } else if (gamepad1.dpad_down) { // Start the sequence to move arm down
                    startedDown = true;
                    movingClaw = true;
                    manipulatorSubsystem.pitchClawToIntake();
                    timer.reset();
                }
            }

            telemetry.addData("Claw position",
                    manipulatorSubsystem.claw.getPosition());
            telemetry.addData("Claw Pitch position",
                    manipulatorSubsystem.clawPitch.getPosition());
            telemetry.addData("Arm power",
                    manipulatorSubsystem.motor.getPower());
            telemetry.addLine();
            telemetry.addData("Claw moving", movingClaw);
            telemetry.addData("Claw Pitch moving", movingClawPitch);
            telemetry.addData("Arm moving", movingArm);

            telemetry.update();
        }
    }
}
