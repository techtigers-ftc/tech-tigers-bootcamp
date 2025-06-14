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
        MoveUpCommand moveUpCommand =
                new MoveUpCommand(manipulatorSubsystem, telemetry);
        MoveDownCommand moveDownCommand =
                new MoveDownCommand(manipulatorSubsystem, telemetry);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        manipulatorSubsystem.openClaw();
        manipulatorSubsystem.pitchClawToIntake();
        timer.reset();

        while (opModeIsActive()) {
            if (!moveUpCommand.isFinished()) { // If the sequence to move arm // up has started
                moveUpCommand.execute();
            } else if (!moveDownCommand.isFinished()) { // If the sequence to move arm down has started
                moveDownCommand.execute();
            } else if (gamepad1.dpad_up) { // If Dpad Up, start action to move up
                moveUpCommand.initialize();
            } else if (gamepad1.dpad_down) { // If Dpad Down, start action to move down
                moveDownCommand.initialize();
            }

            telemetry.addData("Claw position",
                    manipulatorSubsystem.claw.getPosition());
            telemetry.addData("Claw Pitch position",
                    manipulatorSubsystem.clawPitch.getPosition());
            telemetry.addData("Arm power",
                    manipulatorSubsystem.motor.getPower());

            telemetry.update();
        }
    }
}
