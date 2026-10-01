package org.firstinspires.ftc.teamcode.structure;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.algorithm.LLPoseMapper;
import org.firstinspires.ftc.teamcode.algorithm.Scorer;
import org.firstinspires.ftc.teamcode.config.DetectionLabels;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Cluster is a data structure used to store attributes of a cluster of detected objects.
 * (mainly balls)
 *
 * @author cycion
 */
/*
Do we really need this or is "select the nearest balls" better?
 */
public class Cluster {
    public ArrayList<LLResultTypes.DetectorResult> objectStorage;

    /**
     * The area of the cluster in percentage of the total area of the image.
     */
    private double totalArea;

    /**
     * sumX, sumY are the running sum of the center (tx,ty) coordinates of each objects
     * in degrees from the X,Y axis of the image center.
     */
    private double sumX, sumY;

    /**
     * Running total count of the number of pollen, red nectar, and blue nectar.
     */
    private int pollenCount, redNectarCount, blueNectarCount;

    public Cluster() {
        objectStorage = new ArrayList<>();

        totalArea = 0;
        pollenCount = 0;
        redNectarCount = 0;
        blueNectarCount = 0;
        sumX = 0;
        sumY = 0;
    }

    /**
     * This method adds an object to the cluster and updates all attributes.
     * @param object The detection result to add to the cluster.
     */
    public void add(LLResultTypes.DetectorResult object) {
        if (object == null ||
                !Arrays.asList(DetectionLabels.BALLS).contains(object.getClassName())) {
            return;
        }

        objectStorage.add(object);
        totalArea += object.getTargetArea();
        sumX += object.getTargetXDegrees();
        sumY += object.getTargetYDegrees();
        updateLabelCount(object.getClassName(), 1);
    }

    /**
     * Removes the object at the specified index and recalculates all cluster attributes.
     * @param index Index of the object to remove.
     * @return The removed DetectorResult object, or null if the index is out of bounds.
     */
    public LLResultTypes.DetectorResult removeAtPosition(int index) {
        if (index < 0 || index >= objectStorage.size()) {
            return null;
        }

        LLResultTypes.DetectorResult removed = objectStorage.remove(index);
        totalArea -= removed.getTargetArea();
        sumX -= removed.getTargetXDegrees();
        sumY -= removed.getTargetYDegrees();
        updateLabelCount(removed.getClassName(), -1);

        return removed;
    }

    /**
     * Helper functions to update cluster attributes
     * @param className label of the detected object
     * @param delta difference between previous and current count of ball types
     */
    private void updateLabelCount(String className, int delta) {
        if (className == null) {
            return;
        }
        switch (className) {
            case DetectionLabels.POLLEN:
                pollenCount += delta;
                break;
            case DetectionLabels.RED_NECTAR:
                redNectarCount += delta;
                break;
            case DetectionLabels.BLUE_NECTAR:
                blueNectarCount += delta;
                break;
            default:
                break;
        }
    }

    public Pose getLocalizedCenter(Pose robotPose, double turretAngle) {
        return LLPoseMapper.limelight2PedroPose(getXCenter(), getYCenter(), robotPose, turretAngle);
    }

    public double getDistance(Pose robotPose, double turretAngle) {
        return LLPoseMapper.distanceFromRobot(robotPose, getXCenter(), getYCenter());
    }

    /**
     * Rate the cluster using the provided scorer method.
     * @param scorer The scoring algorithm to use.
     * @param robotPose The current Pedro Pathing's Pose of the robot, parameter to calculate distance
     * @param turretAngle The current angle of the turret in radians, parameter to calculate distance
     * @return The score of the cluster. (Higher means better)
     */
    public double score(Scorer scorer, Pose robotPose, double turretAngle) {
        double distance = getDistance(robotPose, turretAngle);
        return scorer.score(this, robotPose, turretAngle);
    }

    public double getArea() {
        return totalArea;
    }

    public int getBallCount() {
        return pollenCount + redNectarCount + blueNectarCount;
    }

    public int getPollenCount() {
        return pollenCount;
    }

    public int getRedNectarCount() {
         return redNectarCount;
    }

    public int getBlueNectarCount() {
        return blueNectarCount;
    }

    public double getXCenter() {
        return sumX / getBallCount();
    }

    public double getYCenter() {
        return sumY / getBallCount();
    }
}
