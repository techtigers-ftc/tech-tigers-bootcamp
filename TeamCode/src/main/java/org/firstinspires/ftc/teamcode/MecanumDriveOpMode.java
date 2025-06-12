package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode contains code for driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * Also note that it is critical to set the correct rotation direction for each motor.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 */
@TeleOp(name = "Mecanum Drive OpMode", group = "Linear OpMode")
public class MecanumDriveOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Any code before this point runs once ONCE when you press INIT
        waitForStart();

        // Any code after this point runs ONCE after you press START

        // Any code in this loop runs REPEATEDLY until the driver presses STOP
        while (opModeIsActive()) {
            // Left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            driveSubsystem.drive(axial, lateral, yaw);
        }
        // Any code after the while loop will run ONCE after the driver presses STOP
    }
}
