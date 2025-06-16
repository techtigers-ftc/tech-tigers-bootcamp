package org.firstinspires;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class OdometrySubsystem {

    private Pose2D startPose;
    private final GoBildaPinpointDriver odo;
    private Pose2D robotPose;
    private Pose2D robotVelocity;
    private RobotState robotState;



    /**
     * Initializes a new OdometrySubsystem.
     *
     * @param hardwareMap The hardware map, used to get hardware references
     * @param startPose   The starting pose of the robot.
     */
    public OdometrySubsystem(HardwareMap hardwareMap, RobotState robotState, Pose2D startPose) {
        this.startPose = startPose;
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
        odo.setOffsets(56, 0);

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


    }

    public void periodic() {
        // Update the odometry position
        odo.update();

        // Get the current position and heading velocities

        double heading = odo.getHeading();
        double x = odo.getPosX();
        double y = odo.getPosY();
        double xVelocity = odo.getVelX();
        double yVelocity = odo.getVelY();
        double headingVelocity = odo.getHeadingVelocity();

        // Get the current position in inches and degrees

        robotPose = new Pose2D(DistanceUnit.INCH, x/25.4, y/25.4, AngleUnit.RADIANS, heading);

        robotVelocity = new Pose2D(DistanceUnit.INCH, xVelocity/25.4, yVelocity/25.4, AngleUnit.RADIANS, headingVelocity);

        robotState.setRobotPose(robotPose);
        robotState.setRobotVelocity(robotVelocity);
    }

    public Pose2D getCurrentPose() {
        return robotPose;
    }

    public Pose2D getCurrentVelocity() {
        return robotVelocity;
    }
}
