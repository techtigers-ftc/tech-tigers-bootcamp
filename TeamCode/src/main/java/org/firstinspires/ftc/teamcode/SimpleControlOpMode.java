package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// This is a simple control OpMode that allows for basic operations of a claw and arm.
@TeleOp(name="Simple Control OpMode", group="Linear OpMode")
public class SimpleControlOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw_servo");
        Servo clawPitch = hardwareMap.servo.get( "claw_pitch_servo");
        DcMotor motor = hardwareMap.dcMotor.get("arm_motor");

        // This sets the motor to brake when no power is applied, which is
        // useful for holding the arm in place when not moving.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Claw open/close operation
            if (gamepad1.a) {
                claw.setPosition(0);
            } else if (gamepad1.b) {
                claw.setPosition(1);
            }

            // Claw Pitch operation
            if (gamepad1.x) {
                clawPitch.setPosition(0);
            } else if (gamepad1.y) {
                clawPitch.setPosition(1);
            }

            // Arm operation
            // Decreased sensitivity so the arm doesn't go flying off; tweak as needed
            double armPower = -0.25 * gamepad1.left_stick_y;
            motor.setPower(armPower);
        }
    }
}
