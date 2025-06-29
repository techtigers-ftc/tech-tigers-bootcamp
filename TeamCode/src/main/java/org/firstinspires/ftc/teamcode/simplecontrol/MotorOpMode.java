package org.firstinspires.ftc.teamcode.simplecontrol;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// This is a simple OpMode that allows for basic operations of a motor, both
// using the encoder for precise movements and manual control using the gamepad.
@TeleOp(name = "Motor OpMode", group = "Linear OpMode")
public class MotorOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("arm_motor");

        // This sets the motor to brake when no power is applied, which is
        // useful for holding the arm in place when not moving.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // This is necessary to reset the motor's encoder position
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motor to run without encoders initially
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motor.setTargetPosition(5000); // Move arm to position 1000
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.75);
            } else if (gamepad1.dpad_down) {
                motor.setTargetPosition(2000); // Move arm back to position 0
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.75);
            }

            if (!motor.isBusy()) {
                // Manual Arm operation
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double armPower = -0.75 * gamepad1.left_stick_y;
                motor.setPower(armPower);
            }

            telemetry.addData("Arm power", motor.getPower());
            telemetry.addData("Arm position", motor.getCurrentPosition());

            telemetry.update();
        }
    }
}
