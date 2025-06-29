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

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Opens the claw.
     */
    public void openClaw() {
        claw.setPosition(0.15);
    }

    /**
     * Closes the claw.
     */
    public void closeClaw() {
        claw.setPosition(0);
    }

    /**
     * Pitches the claw to the drop position.
     */
    public void pitchClawToDrop() {
        clawPitch.setPosition(0.8);
    }

    /**
     * Pitches the claw to the intake position.
     */
    public void pitchClawToIntake() {
        clawPitch.setPosition(0.35);
    }

    /**
     * Moves the arm to the drop position
     */
    public void moveArmToDrop() {
        motor.setTargetPosition(2000);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.75);
    }

    /**
     * Moves the arm to the intake position
     */
    public void moveArmToIntake() {
        motor.setTargetPosition(4800);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.75);
    }

    /**
     * Checks if the arm motor is currently busy (i.e., moving to a target position).
     *
     * @return true if the arm motor is busy, false otherwise.
     */
    public boolean isArmBusy() {
        return motor.isBusy();
    }
}
