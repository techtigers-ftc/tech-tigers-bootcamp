package org.firstinspires.ftc.teamcode.compoundcontrol;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final RobotState robotState;

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
        System.out.println("Red: " + red + ", Blue: " + blue + ", Green: " + green);
    }
}
