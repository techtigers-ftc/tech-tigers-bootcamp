package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.MoveSweeperCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SweeperSubsystem;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name = "Color Control OpMode", group = "Linear OpMode")
public class ColorControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();
        SweeperSubsystem manipulatorSubsystem = new SweeperSubsystem(hardwareMap);
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);

        Command currentCommand = null;
        waitForStart();

        manipulatorSubsystem.resetSweeper();

        while (opModeIsActive()) {
            // Calls the periodic of the sensor subsystem to update sensor values
            sensorSubsystem.periodic();

            // Gets the red and blue values from the robot state
            int red = robotState.getRed();
            int green = robotState.getGreen();
            int blue = robotState.getBlue();


            // Checks if there is an active command that is scheduled.
            if (currentCommand == null) {
                // Check for gamepad input and start a command
                if (gamepad1.a) {
                    // Start the shoot command
                    currentCommand = new ShootCommand(manipulatorSubsystem, driveSubsystem);
                    currentCommand.initialize();
                } else if (gamepad1.left_bumper) {
                    // Moves the sweeper to the "LEFT" position
                    currentCommand = new MoveSweeperCommand(manipulatorSubsystem, "LEFT", 0.2);
                    currentCommand.initialize();
                } else if (gamepad1.right_bumper) {
                    // Moves the sweeper to the "RIGHT" position
                    currentCommand = new MoveSweeperCommand(manipulatorSubsystem, "RIGHT", 0.2);
                    currentCommand.initialize();
                } else {
                    // No command is active and no commands were scheduled so we can allow for
                    // manual driving.

                    double axial = -gamepad1.left_stick_y;
                    double lateral = gamepad1.left_stick_x;
                    double yaw = gamepad1.right_stick_x;

                    driveSubsystem.drive(axial, lateral, yaw);
                }
            } else if (!currentCommand.isFinished()) {
                // There is an active command, so we need to execute it
                currentCommand.execute();
            } else {
                // The current command is finished, so we need to clear it out
                currentCommand = null;
            }

            telemetry.addData("Red value", red);
            telemetry.addData("Green value", green);
            telemetry.addData("Blue value", blue);
            telemetry.addLine();

            telemetry.update();
        }
    }
}
