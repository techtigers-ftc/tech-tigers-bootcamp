package org.firstinspires;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A class representing the state of the robot.
 */
public class RobotState {
    private Pose2D robotPose;
    private Pose2D robotVelocity;

    /**
     * Constructs a RobotState with default values.
     * The robot's initial pose is set to (0, 0) with a heading of 0 radians,
     * and the velocity is also initialized to (0, 0) with a heading of 0 radians.
     */
    public RobotState() {
        this.robotPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);
        this.robotVelocity = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);
    }

    /**
     * @return The current pose of the robot.
     */
    public Pose2D getRobotPose() {
        return robotPose;
    }

    /**
     * Sets the current pose of the robot.
     *
     * @param robotPose The new pose of the robot.
     */
    public void setRobotPose(Pose2D robotPose) {
        this.robotPose = robotPose;
    }

    /**
     * @return The current velocity of the robot.
     */
    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }

    /**
     * Sets the current velocity of the robot.
     *
     * @param robotVelocity The new velocity of the robot.
     */
    public void setRobotVelocity(Pose2D robotVelocity) {
        this.robotVelocity = robotVelocity;
    }
}
