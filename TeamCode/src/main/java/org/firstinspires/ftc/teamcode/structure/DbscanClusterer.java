package org.firstinspires.ftc.teamcode.structure;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.algorithm.ClusterScoring;
import org.firstinspires.ftc.teamcode.algorithm.LLPoseMapper;
import org.firstinspires.ftc.teamcode.config.DetectionLabels;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * DbscanClusterer is a data structure to map detected objects into clusters using DBSCAN.
 */
// I wish Java had a library for this
@Configurable
public class DbscanClusterer {
    // better name would be "Clusterfuck"
    /**
     * Epsilon is the maximum distance between two points for them to be considered neighbors
     * in inches.
     */
    @Sorter(sort = 0)
    public static double eps = 6.0;

    /**
     * MinSamples is the minimum number of points in a neighborhood for a point to be considered
     */
    @Sorter(sort = 1)
    public static int minSamples = 1;

    /**
     * clustersMap stores the clusters
     */
    public final List<Cluster> clustersMap;

    public DbscanClusterer(LLResult llResult) {
        this(llResult, new Pose(0, 0, 0), 0.0);
    }

    public DbscanClusterer(LLResult llResult, Pose robotPose, double turretAngle) {
        this(llResult.getDetectorResults(), robotPose, turretAngle);
    }

    public DbscanClusterer(List<LLResultTypes.DetectorResult> detectorResults) {
        this(detectorResults, new Pose(0, 0, 0), 0.0);
    }

    /**
     * This constructor processes Limelight Detector Result
     * and sort the detected objects into clusters using DBSCAN.
     * @param detectorResults Limelight Detector Result
     * @param robotPose Current Pedro Pose of the robot
     * @param turretAngle Current turret angle
     */
    public DbscanClusterer(List<LLResultTypes.DetectorResult> detectorResults, Pose robotPose, double turretAngle) {
        if (detectorResults == null || detectorResults.isEmpty()) {
            this.clustersMap = new ArrayList<>();
            return;
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
            this.clustersMap = new ArrayList<>();
            return;
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

        this.clustersMap = clusterList;
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

    /**
     * Returns the cluster with the best score
     * @param robotPose Current Pedro Pose of the Robot
     * @param turretAngle Current turret angle in radians
     * @return The best Cluster object
     */
    public Cluster getBestCluster(Pose robotPose, double turretAngle) {
        if (clustersMap.isEmpty()) {
            return null;
        }
        Cluster bestCluster = null;
        double bestScore = -10.0;
        for (Cluster c : clustersMap) {
            double score = c.score(ClusterScoring::score, robotPose, turretAngle);

            if (score > bestScore) {
                bestScore = score;
                bestCluster = c;
            }
        }
        return bestCluster;
    }
}
