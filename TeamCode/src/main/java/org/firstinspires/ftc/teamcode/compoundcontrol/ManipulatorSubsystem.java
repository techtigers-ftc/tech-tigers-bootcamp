package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for controlling a manipulator with a claw and an arm.
 */
public class ManipulatorSubsystem {
    public final Servo claw;
    public final Servo clawPitch;
    public final DcMotor motor;

    /**
     * Constructor for the ManipulatorSubsystem.
     *
     * @param hardwareMap The hardware map to access the robot's hardware components.
     */
    public ManipulatorSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw_servo");
        clawPitch = hardwareMap.servo.get( "claw_pitch_servo");
        motor = hardwareMap.dcMotor.get("arm_motor");

        // This sets the motor to brake when no power is applied, which is
        // useful for holding the arm in place when not moving.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void openClaw() {
        claw.setPosition(0.25);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }

    public void pitchClawToDrop() {
        clawPitch.setPosition(0.5);
    }

    public void pitchClawToIntake() {
        clawPitch.setPosition(0.1);
    }

    public void moveArmToPosition(int position, double power) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(position);
        motor.setPower(power);
    }

    public boolean isArmBusy() {
        return motor.isBusy();
    }
}
