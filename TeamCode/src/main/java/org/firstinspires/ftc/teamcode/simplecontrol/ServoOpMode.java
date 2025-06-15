package org.firstinspires.ftc.teamcode.simplecontrol;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

// This is a simple control OpMode that allows for basic operations of servos
// on a claw
@TeleOp(name="Servo OpMode", group="Linear OpMode")
public class ServoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw_servo");
        Servo clawPitch = hardwareMap.servo.get( "claw_pitch_servo");

        waitForStart();

        // Start servos AFTER start is pressed
        claw.setPosition(0.15);
        clawPitch.setPosition(0.2);

        while (opModeIsActive()) {
            // Claw open/close operation
            if (gamepad1.a) {
                claw.setPosition(0);
            } else if (gamepad1.b) {
                claw.setPosition(0.15);
            }

            // Claw Pitch operation
            if (gamepad1.x) {
                clawPitch.setPosition(0.2);
            } else if (gamepad1.y) {
                clawPitch.setPosition(0.5);
            }

            telemetry.addData("Claw position", claw.getPosition());
            telemetry.addData("Claw Pitch position", clawPitch.getPosition());

            telemetry.update();
        }
    }
}
