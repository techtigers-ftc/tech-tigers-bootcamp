package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simple OpMode that only controls the sweeper servo.
 * This is intended to demonstrate basic servo control.
 */
@TeleOp(name="Sweeper OpMode", group="Linear OpMode")
public class SweeperOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware
        // The string here must correspond to the names of the servo in your robot config
        Servo servo = hardwareMap.servo.get("sweeper_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Any code before this point runs once ONCE when you press INIT
        waitForStart();

        // Any code after this point runs ONCE after you press START

        // Start servos AFTER start is pressed
        servo.setPosition(0.5);

        // Any code in this loop runs REPEATEDLY until the driver presses STOP
        while (opModeIsActive()) {
            // Claw open/close operation
            if (gamepad1.a) {
                servo.setPosition(0.3);
            } else if (gamepad1.b) {
                servo.setPosition(0.7);
            }

            telemetry.addData("Servo Position", servo.getPosition());

            telemetry.update();
        }
        // Any code after the while loop will run ONCE after the driver presses STOP
    }
}
