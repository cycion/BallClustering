package org.firstinspires.ftc.teamcode.algorithm;

import static org.firstinspires.ftc.teamcode.config.CameraConfig.BALL_HEIGHT;
import static org.firstinspires.ftc.teamcode.config.CameraConfig.CAMERA_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.config.CameraConfig.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.config.CameraConfig.CAMERA_SIDEWAYS_OFFSET;
import static org.firstinspires.ftc.teamcode.config.CameraConfig.CAMERA_TILT;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * LLPoseMapper is a utility class to handle the various conversions between Limelight and
 * Pedro Coordinate System
 */

// TODO: Test these methods
public class LLPoseMapper {
    private static final PoseFactory p = PoseFactory.radians();
    /**
     * Convert the Limelight coordinates to Pedro Coordinates
     * @param tx Limelight object tx coordinate
     * @param ty Limelight object ty coordinate
     * @param robotPose Current Pedro Pathing's Pose of the robot
     * @param turretAngle Current angle of the turret in radians
     * @param objectHeight Height of the object in inches
     * @return Pose of the object in Pedro Coordinates
     */
    /*
    (thanks Khánh)
    One problem with this is that tY jumps around a lot, which makes it hard to get a good
    idea of how far the thing is. Fuck.

    Should I Kalman filter this with the total area to better the accuracy?
     */
    public static Pose limelight2PedroPose(double tx, double ty, Pose robotPose, double turretAngle, double objectHeight) {
        double botHeading = robotPose.heading();
        /*
        First, calculate the forward and lateral distances from the camera lens' center ray
        to the center of the cluster.
         */
        double forwardDist = (objectHeight - CAMERA_HEIGHT)
                / Math.tan(Math.toRadians(CAMERA_TILT + ty));
        double lateralDist = forwardDist * Math.tan(Math.toRadians(tx));

        /*
        Second, localize the camera position relative to the field position.
         */

        // Replace cameraHeading variable with the commented-out version if
        // the camera is fixed on the robot itself

        // double cameraHeading = botHeading + CAMERA_YAW;
        double cameraHeading = botHeading + turretAngle;
        double cameraX = robotPose.x() + (CAMERA_FORWARD_OFFSET * Math.cos(botHeading) +
                CAMERA_SIDEWAYS_OFFSET * Math.sin(botHeading));
        double cameraY = robotPose.y() + (CAMERA_FORWARD_OFFSET * Math.sin(botHeading) -
                CAMERA_SIDEWAYS_OFFSET * Math.cos(botHeading));

        /*
        Third, calculate the target's field position
         */
        double targetX = cameraX + forwardDist * Math.cos(cameraHeading) + lateralDist * Math.sin(cameraHeading);
        double targetY = cameraY + forwardDist * Math.sin(cameraHeading) - lateralDist * Math.cos(cameraHeading);
        double targetHeading = Math.atan2(targetY - robotPose.y(), targetX - robotPose.x());

        return p.of(targetX, targetY, targetHeading);
    }

    /**
     * Convert the Limelight coordinates to Pedro Coordinates, with default ball object height
     * @param tx Limelight object tx coordinate
     * @param ty Limelight object ty coordinate
     * @param robotPose Current Pedro Pathing's Pose of the robot
     * @param turretAngle Current angle of the turret in radians
     * @return Pose of the object in Pedro Coordinates
     */
    public static Pose limelight2PedroPose(double tx, double ty, Pose robotPose, double turretAngle) {
        return limelight2PedroPose(tx, ty, robotPose, turretAngle, BALL_HEIGHT);
    }

    /**
     * Calculate the Euclidean distance from the robot to the target in inches
     * @param robotPose Current Pedro Pathing's Pose of the robot
     * @param targetPose Pose of the target in Pedro Coordinates
     * @return Distance from the robot to the target in inches
     */
    public static double distanceFromRobot(Pose robotPose, Pose targetPose) {
        return Math.hypot(robotPose.x() - targetPose.x(), robotPose.y() - targetPose.y());
    }

    /**
     * Calculate the Euclidean distance from the robot to the target in inches
     * @param robotPose Current Pedro Pathing's Pose of the robot
     * @param tx Limelight object tx coordinate
     * @param ty Limelight object ty coordinate
     * @return Distance from the robot to the target in inches
     */
    public static double distanceFromRobot(Pose robotPose, double tx, double ty) {
        Pose targetPose = limelight2PedroPose(tx, ty, robotPose, 0);
        return distanceFromRobot(robotPose, targetPose);
    }
}
