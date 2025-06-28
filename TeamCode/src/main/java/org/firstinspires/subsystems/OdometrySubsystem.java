package org.firstinspires.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.GoBildaPinpointDriver;
import org.firstinspires.RobotState;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * A subsystem for managing odometry using the GoBILDA Pinpoint odometry system.
 * This subsystem tracks the robot's position and velocity in a 2D space.
 */
public class OdometrySubsystem {
    private final GoBildaPinpointDriver odo;
    private final RobotState robotState;

    /**
     * Initializes a new OdometrySubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param startPose   The starting pose of the robot.
     */
    public OdometrySubsystem(HardwareMap hardwareMap, RobotState robotState, Pose2D startPose) {
        this.robotState = robotState;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(56, 0, DistanceUnit.MM);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
//        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        startPose = new Pose2D(DistanceUnit.INCH,startPose.getX(DistanceUnit.INCH),
                startPose.getY(DistanceUnit.INCH),AngleUnit.DEGREES, startPose.getHeading(AngleUnit.DEGREES));
        odo.setPosition(startPose);

    }

    public void periodic() {
        // Update the odometry position
        odo.update();

        // Get the current position and heading velocities

        double heading = odo.getHeading(AngleUnit.RADIANS);
        double x = odo.getPosX(DistanceUnit.INCH);
        double y = odo.getPosY(DistanceUnit.INCH);
        double headingVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        double xVelocity = odo.getVelX(DistanceUnit.INCH);
        double yVelocity = odo.getVelY(DistanceUnit.INCH);

        // Get the current position in inches and degrees

        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, x, y,
                AngleUnit.RADIANS, heading);

        Pose2D robotVelocity = new Pose2D(DistanceUnit.INCH, xVelocity,
                yVelocity, AngleUnit.RADIANS, headingVelocity);

        robotState.setRobotPose(robotPose);
        robotState.setRobotVelocity(robotVelocity);
    }
}
