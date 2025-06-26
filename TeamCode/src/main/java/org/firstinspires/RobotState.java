package org.firstinspires;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotState {
    private Pose2D robotPose;
    private Pose2D robotVelocity;

    public RobotState() {
        this.robotPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);
        this.robotVelocity = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);
    }

    public Pose2D getRobotPose() {
        return robotPose;
    }
    public void setRobotPose(Pose2D robotPose) {
        this.robotPose = robotPose;
    }
    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }
    public void setRobotVelocity(Pose2D robotVelocity) {
        this.robotVelocity = robotVelocity;
    }
}
