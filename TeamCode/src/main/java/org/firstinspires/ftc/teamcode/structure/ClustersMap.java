package org.firstinspires.ftc.teamcode.structure;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.algorithm.LLPoseMapper;
import org.firstinspires.ftc.teamcode.config.DetectionLabels;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * ClustersMap is a data structure to map detected objects into clusters using DBSCAN.
 */
/*
I wish Java had a library for this.
 */
@Configurable
public class ClustersMap {
    @Sorter(sort = 0)
    public static double defaultEps = 6.0;

    @Sorter(sort = 1)
    public static int defaultMinSamples = 1;

    private double eps;
    private int minSamples;
    private List<Cluster> clusters;

    public ClustersMap() {
        this(defaultEps, defaultMinSamples);
    }

    public ClustersMap(double eps, int minSamples) {
        this.eps = eps;
        this.minSamples = minSamples;
        this.clusters = new ArrayList<>();
    }

    /**
     * Clusters the detected balls in an LLResult using DBSCAN with default robot pose (0, 0, 0) and turret angle 0.
     *
     * @param llResult Limelight result containing detector results.
     * @return List of generated Clusters.
     */
    public List<Cluster> cluster(LLResult llResult) {
        return cluster(llResult, new Pose(0, 0, 0), 0.0);
    }

    /**
     * Clusters the detected balls in an LLResult using DBSCAN given the robot pose and turret angle.
     *
     * @param llResult Limelight result containing detector results.
     * @param robotPose Current Pedro Pathing robot Pose.
     * @param turretAngle Current turret angle in radians.
     * @return List of generated Clusters.
     */
    public List<Cluster> cluster(LLResult llResult, Pose robotPose, double turretAngle) {
        if (llResult == null) {
            this.clusters = new ArrayList<>();
            return this.clusters;
        }
        return cluster(llResult.getDetectorResults(), robotPose, turretAngle);
    }

    /**
     * Clusters a list of DetectorResults using DBSCAN with default robot pose (0, 0, 0) and turret angle 0.
     *
     * @param detectorResults List of detector results from Limelight.
     * @return List of generated Clusters.
     */
    public List<Cluster> cluster(List<LLResultTypes.DetectorResult> detectorResults) {
        return cluster(detectorResults, new Pose(0, 0, 0), 0.0);
    }

    /**
     * Clusters a list of DetectorResults using DBSCAN given the robot pose and turret angle.
     *
     * @param detectorResults List of detector results from Limelight.
     * @param robotPose Current Pedro Pathing robot Pose.
     * @param turretAngle Current turret angle in radians.
     * @return List of generated Clusters.
     */
    public List<Cluster> cluster(List<LLResultTypes.DetectorResult> detectorResults, Pose robotPose, double turretAngle) {
        if (detectorResults == null || detectorResults.isEmpty()) {
            this.clusters = new ArrayList<>();
            return this.clusters;
        }

        // Filter out results that are not valid balls
        List<LLResultTypes.DetectorResult> validBalls = new ArrayList<>();
        List<String> ballLabels = Arrays.asList(DetectionLabels.BALLS);
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            if (dr != null && dr.getClassName() != null && ballLabels.contains(dr.getClassName())) {
                validBalls.add(dr);
            }
        }

        int n = validBalls.size();
        if (n == 0) {
            this.clusters = new ArrayList<>();
            return this.clusters;
        }

        Pose effectiveRobotPose = (robotPose != null) ? robotPose : new Pose(0, 0, 0);

        // Precompute 2D field/camera positions in inches
        Pose[] positions = new Pose[n];
        for (int i = 0; i < n; i++) {
            LLResultTypes.DetectorResult dr = validBalls.get(i);
            positions[i] = LLPoseMapper.limelight2PedroPose(
                    dr.getTargetXDegrees(),
                    dr.getTargetYDegrees(),
                    effectiveRobotPose,
                    turretAngle
            );
        }

        // Run DBSCAN
        boolean[] visited = new boolean[n];
        boolean[] inCluster = new boolean[n];
        List<Cluster> clusterList = new ArrayList<>();

        for (int k = 0; k < n; k++) {
            if (visited[k]) {
                continue;
            }
            visited[k] = true;

            List<Integer> neighbors = getNeighbors(k, positions, eps);

            if (neighbors.size() < minSamples) {
                // Point is noise under current minSamples configuration
                continue;
            }

            Cluster cluster = new Cluster();
            cluster.add(validBalls.get(k));
            inCluster[k] = true;

            List<Integer> seeds = new ArrayList<>(neighbors);

            int index = 0;
            while (index < seeds.size()) {
                int currentPt = seeds.get(index++);

                if (!visited[currentPt]) {
                    visited[currentPt] = true;
                    List<Integer> currentNeighbors = getNeighbors(currentPt, positions, eps);
                    if (currentNeighbors.size() >= minSamples) {
                        for (int neighbor : currentNeighbors) {
                            if (!seeds.contains(neighbor)) {
                                seeds.add(neighbor);
                            }
                        }
                    }
                }

                if (!inCluster[currentPt]) {
                    cluster.add(validBalls.get(currentPt));
                    inCluster[currentPt] = true;
                }
            }

            clusterList.add(cluster);
        }

        this.clusters = clusterList;
        return clusterList;
    }

    private List<Integer> getNeighbors(int ptIndex, Pose[] positions, double eps) {
        List<Integer> neighbors = new ArrayList<>();
        Pose p1 = positions[ptIndex];
        for (int j = 0; j < positions.length; j++) {
            Pose p2 = positions[j];
            double dist = Math.hypot(p1.x() - p2.x(), p1.y() - p2.y());
            if (dist <= eps) {
                neighbors.add(j);
            }
        }
        return neighbors;
    }

    public List<Cluster> getClusters() {
        return clusters;
    }

    public double getEps() {
        return eps;
    }

    public void setEps(double eps) {
        this.eps = eps;
    }

    public int getMinSamples() {
        return minSamples;
    }

    public void setMinSamples(int minSamples) {
        this.minSamples = minSamples;
    }
}
