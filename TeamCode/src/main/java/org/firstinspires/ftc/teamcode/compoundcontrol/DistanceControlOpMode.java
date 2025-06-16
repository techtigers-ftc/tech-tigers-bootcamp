package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name="Distance Control OpMode", group="Linear OpMode")
public class DistanceControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotState robotState = new RobotState();
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap, robotState);
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
            // Calls the periodic of the sensor subsystem to update sensor values
            sensorSubsystem.periodic();

            // Gets the distance value from the robot state
            double distance = robotState.getDistance();

            if (!moveUpCommand.isFinished()) { // If the sequence to move arm // up has started
                moveUpCommand.execute();
            } else if (!moveDownCommand.isFinished()) { // If the sequence to move arm down has started
                moveDownCommand.execute();
            } else if (distance < 5) { // Moves up if there is an object within 5 cm of the sensor
                moveUpCommand.initialize();
            } else if (distance > 5) { // Moves the arm down if there is an object more than 5 cm away from the sensor
                moveDownCommand.initialize();
            }


            telemetry.addData("Distance (cm)", distance);
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
