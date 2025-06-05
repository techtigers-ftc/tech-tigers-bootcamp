package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Start servos AFTER start is pressed
        claw.setPosition(0.25);
        clawPitch.setPosition(0.1);

        while (opModeIsActive()) {
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
            double armPower = -0.75 * gamepad1.left_stick_y;
            motor.setPower(armPower);

            telemetry.addData("Claw position", claw.getPosition());
            telemetry.addData("Claw Pitch position", clawPitch.getPosition());
            telemetry.addData("Arm power", motor.getPower());

            telemetry.update();
        }
    }
}
