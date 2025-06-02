package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// This is an opmode that combines control of a claw and an arm with more complex logic.
@TeleOp(name="Compound Control OpMode", group="Linear OpMode")
public class CompoundControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw_servo");
        Servo clawPitch = hardwareMap.servo.get( "claw_pitch_servo");
        DcMotor motor = hardwareMap.dcMotor.get("arm_motor");

        // This sets the motor to brake when no power is applied, which is
        // useful for holding the arm in place when not moving.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This is a timer object. We can use it to measure time across
        // loops, and it is useful for timing operations.
        ElapsedTime timer = new ElapsedTime();

        boolean startedUp = false;
        boolean startedDown = false;
        boolean movingClaw = false;
        boolean movingClawPitch = false;
        boolean movingArm = false;

        waitForStart();

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
                        motor.setPower(0.5); // Move arm up
                        timer.reset();
                    }
                } else { // If the arm is moving
                    if (timer.seconds() > 1) { // Wait for arm to finish moving
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
                        clawPitch.setPosition(0); // Claw pitch to intake
                                                    // position
                        claw.setPosition(0); // Open claw
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
                    claw.setPosition(1); // Close claw
                    timer.reset();
                } else if (gamepad1.dpad_down) { // Start the sequence to move arm down
                    startedDown = true;
                    movingArm = true;
                    motor.setPower(-0.5); // Move arm down
                    timer.reset();
                }
            }

            telemetry.addData("Claw position", claw.getPosition());
            telemetry.addData("Claw Pitch position", clawPitch.getPosition());
            telemetry.addData("Arm power", motor.getPower());
            telemetry.addLine();
            telemetry.addData("Claw moving", movingClaw);
            telemetry.addData("Claw Pitch moving", movingClawPitch);
            telemetry.addData("Arm moving", movingArm);
        }
    }
}
