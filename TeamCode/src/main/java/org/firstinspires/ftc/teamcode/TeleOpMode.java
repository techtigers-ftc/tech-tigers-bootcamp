package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

// This is a full TeleOpMode. It contains code for driving a robot with a
// mecanum drive train as well as controlling a claw, and controlling an arm
// with a pitch mechanism, both with basic control and compound control
// sequences.
// While this code works, it also is very difficult to read and understand,
// which can lead to confusion and bugs.
@TeleOp(name = "TeleOpMode", group = "Linear OpMode")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw_servo");
        Servo clawPitch = hardwareMap.servo.get("claw_pitch_servo");
        DcMotor motor = hardwareMap.dcMotor.get("arm_motor");

        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("left_front");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("left_back");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("right_front");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();

        boolean startedUp = false;
        boolean startedDown = false;
        boolean movingClaw = false;
        boolean movingClawPitch = false;
        boolean movingArm = false;

        waitForStart();

        claw.setPosition(0.25);
        clawPitch.setPosition(0.1);
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
                        clawPitch.setPosition(0.5); // Claw pitch to drop
                        // position
                        timer.reset();
                    }
                } else if (movingClawPitch) { // If the claw pitch is moving
                    if (timer.seconds() > 0.5) {
                        movingClawPitch = false;
                        movingArm = true;
                        motor.setPower(0.75); // Move arm up
                        timer.reset();
                    }
                } else { // If the arm is moving
                    if (timer.seconds() > 1) { // Wait for arm to finish moving
                        motor.setPower(0); // Stop arm motor
                        movingArm = false;
                        startedUp = false; // Reset for next operation
                    }
                }
            } else if (startedDown) { // If the sequence to move arm down has started
                if (movingArm) { // If the arm is moving down
                    if (timer.seconds() > 1) { // Wait for arm to finish moving down
                        movingArm = false;
                        movingClawPitch = true;
                        movingClaw = true;
                        motor.setPower(0); // Stop arm motor
                        clawPitch.setPosition(0.1); // Claw pitch to intake
                        // position
                        claw.setPosition(0.25); // Open claw
                        timer.reset();
                    }
                } else { // If the claw pitch and claw are moving
                    if (timer.seconds() > 0.5) { // Wait for claw pitch and claw to finish moving
                        movingClaw = false;
                        movingClawPitch = false;
                        startedDown = false; // Reset for next operation
                    }
                }
            } else {
                if (gamepad1.dpad_up) { // Start the sequence to move arm up
                    startedUp = true;
                    movingClaw = true;
                    claw.setPosition(0); // Close claw
                    timer.reset();
                } else if (gamepad1.dpad_down) { // Start the sequence to move arm down
                    startedDown = true;
                    movingArm = true;
                    motor.setPower(-0.5); // Move arm down
                    timer.reset();
                }
            }

            // Basic Manipulator Control (Only if not in a sequence)

            // Claw open/close operation
            if (gamepad1.a) {
                claw.setPosition(0);
            } else if (gamepad1.b) {
                claw.setPosition(0.25);
            }

            // Claw Pitch operation
            if (gamepad1.x) {
                clawPitch.setPosition(0.1);
            } else if (gamepad1.y) {
                clawPitch.setPosition(0.5);
            }

            // Arm operation
            // Decreased sensitivity so the arm doesn't go flying off; tweak as needed
            if (!movingArm) {
                double armPower = -0.75 * (gamepad1.right_trigger - gamepad1.left_trigger);
                motor.setPower(armPower);
            }

            // Drive Control
            double max;

            // Left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addLine();
            telemetry.addData("Claw position", claw.getPosition());
            telemetry.addData("Claw Pitch position", clawPitch.getPosition());
            telemetry.addData("Arm power", motor.getPower());
            telemetry.addLine();
            telemetry.addData("Claw moving", movingClaw);
            telemetry.addData("Claw Pitch moving", movingClawPitch);
            telemetry.addData("Arm moving", movingArm);

            telemetry.update();

        }
    }
}
