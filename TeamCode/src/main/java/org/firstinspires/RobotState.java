package org.firstinspires;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A class representing the state of the robot.
 */
public class RobotState {
    private Pose2D currentPose;
    private Pose2D currentVelocity;

    /**
     * Constructs a new RobotState with default values.
     */
    public RobotState() {
        this.currentPose = new Pose2D(DistanceUnit.CM, 0, 0,
                AngleUnit.DEGREES, 0);
        this.currentVelocity = new Pose2D(DistanceUnit.CM, 0, 0,
                AngleUnit.DEGREES, 0);
    }

    /**
     * @return the current pose of the robot.
     */
    public Pose2D getCurrentPose() {
        return currentPose;
    }

    /**
     * Sets the current pose of the robot.
     *
     * @param currentPose the new pose to set.
     */
    public void setCurrentPose(Pose2D currentPose) {
        this.currentPose = currentPose;
    }

    /**
     * @return the current velocity of the robot.
     */
    public Pose2D getCurrentVelocity() {
        return currentVelocity;
    }

    /**
     * Sets the current velocity of the robot.
     *
     * @param currentVelocity the new velocity to set.
     */
    public void setCurrentVelocity(Pose2D currentVelocity) {
        this.currentVelocity = currentVelocity;
    }
}
