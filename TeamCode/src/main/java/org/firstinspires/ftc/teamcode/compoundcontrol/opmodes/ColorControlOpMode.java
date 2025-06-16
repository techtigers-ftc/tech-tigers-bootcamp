package org.firstinspires.ftc.teamcode.compoundcontrol.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.compoundcontrol.subsystems.ManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.compoundcontrol.commands.MoveDownCommand;
import org.firstinspires.ftc.teamcode.compoundcontrol.commands.MoveUpCommand;
import org.firstinspires.ftc.teamcode.compoundcontrol.RobotState;
import org.firstinspires.ftc.teamcode.compoundcontrol.subsystems.SensorSubsystem;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name="Color Control OpMode", group="Linear OpMode")
public class ColorControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();
        ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(hardwareMap);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);
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
            // Calls the periodic of the sensor subsystem to update sensor values
            sensorSubsystem.periodic();

            // Gets the red and blue values from the robot state
           int red = robotState.getRed();
           int blue = robotState.getBlue();


            if (!moveUpCommand.isFinished()) { // If the sequence to move arm // up has started
                moveUpCommand.execute();
            } else if (!moveDownCommand.isFinished()) { // If the sequence to move arm down has started
                moveDownCommand.execute();
            } else if (blue > 1000) { // If the blue is intense enough, move the arm up
                moveUpCommand.initialize();
            } else if (red > 1000) { // If the red is intense enough, move the arm down
                moveDownCommand.initialize();
            }

            telemetry.addData("Red value", red);
            telemetry.addData("Blue value", blue);
            telemetry.addLine();
            telemetry.addData("Claw position",
                    manipulatorSubsystem.claw.getPosition());
            telemetry.addData("Claw Pitch position",
                    manipulatorSubsystem.clawPitch.getPosition());
            telemetry.addData("Arm power",
                    manipulatorSubsystem.motor.getPower());
            telemetry.addData("Arm position",
                    manipulatorSubsystem.motor.getCurrentPosition());

            telemetry.update();
        }
    }
}
