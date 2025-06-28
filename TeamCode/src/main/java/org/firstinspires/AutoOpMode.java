package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.commands.Command;
import org.firstinspires.commands.DriveToPointCommand;
import org.firstinspires.commands.TurnToAngleCommand;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.subsystems.DriveSubsystem;
import org.firstinspires.subsystems.OdometrySubsystem;

@Autonomous
public class AutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();

        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        OdometrySubsystem odometrySubsystem =
                new OdometrySubsystem(hardwareMap, robotState,
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES , 0)
                );

        // TODO: Create a list of commands that moves the robot in a square pattern
        Command[] commands = {
                new DriveToPointCommand(robotState, driveSubsystem,
                        0.5, 0.5, 30, 30),
                new TurnToAngleCommand(robotState, driveSubsystem,
                        0.5, 90)
        };

        waitForStart();

        int commandIndex = 0;
        commands[commandIndex].initialize();

        while (opModeIsActive()) {
            odometrySubsystem.periodic();

            commands[commandIndex].execute();
            if (commands[commandIndex].isFinished()) {
                commands[commandIndex].end();
                commandIndex++;
                if (commandIndex < commands.length) {
                    commands[commandIndex].initialize();
                } else {
                    break;
                }
            }
        }
    }
}
