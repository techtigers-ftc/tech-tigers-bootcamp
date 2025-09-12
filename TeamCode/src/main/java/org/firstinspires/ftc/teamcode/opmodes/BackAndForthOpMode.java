package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.DriveUntilColorCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SensorSubsystem;

@TeleOp(name = "BackAndForthColor", group="Linear OpMode")
public class BackAndForthOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);
        Command currentCommand = null;

        waitForStart();

        while (opModeIsActive()) {
            sensorSubsystem.periodic();

            if (robotState.getRed() > 1000) {
                robotState.setDetectRed(false);
            } else if (robotState.getBlue() > 1000) {
                robotState.setDetectRed(true);
            }

            if (currentCommand == null) {
                double power = robotState.isDetectRed() ? -0.2 : 0.2;
                currentCommand = new DriveUntilColorCommand(driveSubsystem, robotState,
                        power, 1000);
                currentCommand.initialize();
            } else if (!currentCommand.isFinished()) {
                // There is an active command, so we need to execute it
                currentCommand.execute();
            } else {
                // The current command is finished, so we need to clear it out
                driveSubsystem.drive(0, 0, 0);
                currentCommand = null;
            }

            telemetry.addData("Red value: ", robotState.getRed());
            telemetry.addData("Blue value: ", robotState.getBlue());
            telemetry.update();
        }
    }
}
