package org.firstinspires.ftc.teamcode.compoundcontrol.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.compoundcontrol.RobotState;

/**
 * Subsystem for reading sensor values from a color sensor and updating the robot state.
 */
public class SensorSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final RobotState robotState;

    /**
     * Constructor for the SensorSubsystem.
     *
     * @param hardwareMap The hardware map to access the robot's hardware components.
     * @param robotState The robot state to update with sensor values.
     */
    public SensorSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.robotState = robotState;
    }

    /**
     * Method to be called periodically in the opmode to read sensor values and update the robot state
     * with these values.
     */
    public void periodic() {
        // Read color values from the sensor
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        double distance = colorSensor.getDistance(DistanceUnit.CM);

        // Update the robot state with the new color values
        robotState.setRed(red);
        robotState.setBlue(blue);
        robotState.setGreen(green);
        robotState.setDistance(distance);

        // Optionally, you can log the values for debugging
        RobotLog.dd("SensorSubsystem",
                "Red: %d, Blue: %d, Green: %d", red, blue, green);
    }
}
