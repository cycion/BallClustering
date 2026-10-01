package org.firstinspires.ftc.teamcode.algorithm;

import static org.firstinspires.ftc.teamcode.config.GeneralConfig.ALLIANCE_COLOR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.CameraConfig;
import org.firstinspires.ftc.teamcode.structure.Cluster;

/**
 * Contains scoring algorithms used to evaluate clusters of game elements (pollen and nectar)
 * relative to distance, alliance, purity, and angular alignment to determine the optimal cluster
 * for targeting or navigation.
 */
// WARNING: VIBE-CODED
@Configurable
public class ClusterScoring {

    @Sorter(sort = 0)
    public static double ALLY_NECTAR_WEIGHT = 1.65;

    @Sorter(sort = 1)
    public static double POLLEN_WEIGHT = 1.0;

    @Sorter(sort = 2)
    public static double OPPONENT_NECTAR_WEIGHT = 0.0;

    @Sorter(sort = 3)
    public static double OPPONENT_NECTAR_PENALTY = 0.5;

    @Sorter(sort = 4)
    public static double DISTANCE_DECAY_RATE = 0.02;

    @Sorter(sort = 5)
    public static double ALIGNMENT_DECAY_RATE = 0.01;

    @Sorter(sort = 6)
    public static double AREA_WEIGHT = 0.05;

    @Sorter(sort = 7)
    public static double PURITY_EXPONENT = 1.0;

    /**
     * Evaluates the cluster score using the default alliance color from GeneralConfig.
     *
     * @param cluster The cluster object to score.
     * @return A single numerical value representing the cluster score.
     */
    public double score(Cluster cluster) {
        return score(cluster, ALLIANCE_COLOR, null, 0.0);
    }

    /**
     * Evaluates the cluster score for a specific alliance.
     *
     * @param cluster The cluster object to score.
     * @param alliance The alliance color (RED or BLUE).
     * @return A single numerical value representing the cluster score.
     */
    public double score(Cluster cluster, Alliance alliance) {
        return score(cluster, alliance, null, 0.0);
    }

    /**
     * Evaluates the cluster score using default alliance color with robot pose and turret angle.
     *
     * @param cluster The cluster object to score.
     * @param robotPose Current pose of the robot.
     * @param turretAngle Current turret angle in radians.
     * @return A single numerical value representing the cluster score.
     */
    public static double score(Cluster cluster, Pose robotPose, double turretAngle) {
        return score(cluster, ALLIANCE_COLOR, robotPose, turretAngle);
    }

    /**
     * Evaluates the cluster score considering game element point values, alliance, purity,
     * distance, angular alignment (tx), and cluster total area.
     *
     * @param cluster The cluster object to score.
     * @param alliance The alliance color (RED or BLUE).
     * @param robotPose Current pose of the robot (can be null for estimated distance via ty).
     * @param turretAngle Current turret angle in radians.
     * @return A single numerical value representing the cluster score.
     */
    public static double score(Cluster cluster, Alliance alliance, Pose robotPose, double turretAngle) {
        return calculateScore(cluster, alliance, robotPose, turretAngle);
    }

    /**
     * Static helper method to calculate the cluster score taking into account multiple factors:
     * <ul>
     *   <li>Game element point values based on alliance (Ally nectar: 1.65, Pollen: 1.0, Opponent nectar: 0.0)</li>
     *   <li>Opponent element presence penalty</li>
     *   <li>Cluster purity ratio ((ally + pollen) / total)</li>
     *   <li>Distance decay factor (closer clusters score higher)</li>
     *   <li>Alignment decay factor (clusters closer to center ray tx=0 score higher)</li>
     *   <li>Cluster total target area bonus</li>
     * </ul>
     *
     * @param cluster The cluster object to score.
     * @param alliance The alliance color (RED or BLUE).
     * @param robotPose Current pose of the robot (or null if estimating via ty).
     * @param turretAngle Current turret angle in radians.
     * @return A single numerical score value.
     */
    public static double calculateScore(Cluster cluster, Alliance alliance, Pose robotPose, double turretAngle) {
        if (cluster == null || cluster.getBallCount() == 0) {
            return 0.0;
        }

        int allyNectarCount;
        int opponentNectarCount;

        switch (alliance) {
            case BLUE:
                allyNectarCount = cluster.getBlueNectarCount();
                opponentNectarCount = cluster.getRedNectarCount();
                break;
            case RED:
            default:
                allyNectarCount = cluster.getRedNectarCount();
                opponentNectarCount = cluster.getBlueNectarCount();
                break;
        }

        int pollenCount = cluster.getPollenCount();
        int totalBallCount = cluster.getBallCount();

        // 1. Raw Point Value calculation
        double rawPoints = (allyNectarCount * ALLY_NECTAR_WEIGHT)
                + (pollenCount * POLLEN_WEIGHT)
                + (opponentNectarCount * OPPONENT_NECTAR_WEIGHT);

        // 2. Opponent Nectar Penalty
        double penalty = opponentNectarCount * OPPONENT_NECTAR_PENALTY;
        double netPoints = Math.max(0.0, rawPoints - penalty);

        // 3. Cluster Purity Factor
        double purityRatio = (double) (allyNectarCount + pollenCount) / totalBallCount;
        double purityFactor = Math.pow(purityRatio, PURITY_EXPONENT);

        // 4. Distance Calculation and Decay Factor
        double distance;
        if (robotPose != null) {
            distance = cluster.getDistance(robotPose, turretAngle);
        } else {
            // Estimate forward distance from camera ty degrees
            double ty = cluster.getYCenter();
            double forwardDist = (CameraConfig.BALL_HEIGHT - CameraConfig.CAMERA_HEIGHT)
                    / Math.tan(Math.toRadians(CameraConfig.CAMERA_TILT + ty));
            distance = Math.abs(forwardDist);
        }
        double distanceFactor = 1.0 / (1.0 + DISTANCE_DECAY_RATE * distance);

        // 5. Angular Alignment (tx) Decay Factor
        double txAbs = Math.abs(cluster.getXCenter());
        double alignmentFactor = 1.0 / (1.0 + ALIGNMENT_DECAY_RATE * txAbs);

        // 6. Cluster Target Area Bonus
        double areaBonus = cluster.getArea() * AREA_WEIGHT;

        // Final score combination
        double finalScore = (netPoints * purityFactor * distanceFactor * alignmentFactor) + areaBonus;

        return Math.max(0.0, finalScore);
    }
}