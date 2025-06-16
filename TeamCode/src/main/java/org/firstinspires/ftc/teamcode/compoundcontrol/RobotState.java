package org.firstinspires.ftc.teamcode.compoundcontrol;

/**
 * Class that represents the state of the robot, including color and distance readings from the color sensor.
 */
public class RobotState {
    private int red;
    private int blue;
    private int green;
    private double distance;

    /**
     * Constructor for RobotState
     * Initializes color and distance values to zero.
     */
    public RobotState() {
        this.red = 0;
        this.blue = 0;
        this.green = 0;
        this.distance = 0;
    }

    /**
     * @return red value from color sensor
     */
    public int getRed() {
        return red;
    }

    /**
     * Sets red value from color sensor
     * @param red value to set
     */
    public void setRed(int red) {
        this.red = red;
    }

    /**
     * @return blue value from color sensor
     */
    public int getBlue() {
        return blue;
    }

    /**
     * Sets blue value from color sensor
     * @param blue value to set
     */
    public void setBlue(int blue) {
        this.blue = blue;
    }

    /**
     * @return green value from color sensor
     */
    public int getGreen() {
        return green;
    }

    /**
     * Sets green value from color sensor
     * @param green value to set
     */
    public void setGreen(int green) {
        this.green = green;
    }

    /**
     * @return distance value from distance sensor
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Sets distance value from distance sensor
     * @param distance value to set
     */
    public void setDistance(double distance) {
        this.distance = distance;
    }
}
