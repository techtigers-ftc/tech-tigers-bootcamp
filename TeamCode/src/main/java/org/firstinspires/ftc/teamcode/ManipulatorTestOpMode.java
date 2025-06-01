package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Manipulator Test OpMode", group="Linear OpMode")
public class ManipulatorTestOpMode extends LinearOpMode {
    private Servo claw;
    private Servo clawPitch;
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw_servo");
        clawPitch = hardwareMap.get(Servo.class, "claw_pitch_servo");
        motor = hardwareMap.get(DcMotor.class, "arm_motor");
        waitForStart();

        while (opModeIsActive()) {
            // Claw open/close operation
            if (gamepad1.a) {
                claw.setPosition(0);
            } else if (gamepad1.b) {
                claw.setPosition(0.33);
            } else if (gamepad1.x) {
                claw.setPosition(0.66);
            } else if (gamepad1.y) {
                claw.setPosition(1);
            }

            // Claw pitch operation
            if (gamepad1.dpad_up) {
                clawPitch.setPosition(clawPitch.getPosition() + 5);
            } else if (gamepad1.dpad_down) {
                clawPitch.setPosition(clawPitch.getPosition() - 5);
            }

            // Arm operation
            // Decreased sensitivity so the arm doesn't go flying off; tweak as needed
            motor.setPower(-0.25*gamepad1.left_stick_y);
        }
    }
}
