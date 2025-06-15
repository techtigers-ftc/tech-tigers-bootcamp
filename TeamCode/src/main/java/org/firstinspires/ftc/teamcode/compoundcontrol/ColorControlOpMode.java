package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name="Color Control OpMode", group="Linear OpMode")
public class ColorControlOpMode extends LinearOpMode {
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
            // Moving things based on sensor input
            RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
            double red = colorSensor.red();
            double blue = colorSensor.blue();

            if (!moveUpCommand.isFinished()) { // If the sequence to move arm // up has started
                moveUpCommand.execute();
            } else if (!moveDownCommand.isFinished()) { // If the sequence to move arm down has started
                moveDownCommand.execute();
            } else if (blue > 1000) {
                moveUpCommand.initialize();
            } else if (red > 1000) {
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
