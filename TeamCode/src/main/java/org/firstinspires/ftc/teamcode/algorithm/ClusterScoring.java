package org.firstinspires.ftc.teamcode.algorithm;

import static org.firstinspires.ftc.teamcode.config.GeneralConfig.ALLIANCE_COLOR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * Contains scoring algorithms used to evaluate clusters of game elements (pollen and nectar)
 * relative to distance, and to determine the optimal cluster for targeting or navigation.
 */

@Configurable
public class ClusterScoring {
    // TODO: Calibrate these weights
    // TODO: Create better algorithms for scoring
    @Sorter(sort = 0)
    public static double pollenWeight = 1;
    @Sorter(sort = 1)
    public static double allyNectarWeight = 1.65;
    @Sorter(sort = 2)
    public static double opponentNectarWeight = -2;
    @Sorter(sort = 3)
    public static double distanceWeight = -0.5;

    /**
     * Calculates a weighted score for a cluster based on counts of pollen, ally nectar,
     * opponent nectar, and distance from the robot, adjusted by respective configurable weights.
     *
     * @param pollen      number of pollen elements in the cluster
     * @param redNectar   number of red nectar elements in the cluster
     * @param blueNectar  number of blue nectar elements in the cluster
     * @param distance    distance from the robot to the cluster
     * @return the calculated weighted score
     */
    public double weightedScore(int pollen, int redNectar, int blueNectar, double distance) {
        int opponentNectarCount = (ALLIANCE_COLOR == Alliance.RED) ? blueNectar : redNectar;
        int allyNectarCount = (ALLIANCE_COLOR == Alliance.RED) ? redNectar : blueNectar;

        return pollen * pollenWeight +
                allyNectarCount * allyNectarWeight +
                opponentNectarCount * opponentNectarWeight +
                distance * distanceWeight;
    }

    /**
     * Calculates a score based on total count of alliance elements (pollen + ally nectar)
     * and distance penalty.
     *
     * @param pollen      number of pollen elements in the cluster
     * @param redNectar   number of red nectar elements in the cluster
     * @param blueNectar  number of blue nectar elements in the cluster
     * @param distance    distance from the robot to the cluster
     * @return the calculated ball count score
     */
    public double ballCountScore(int pollen, int redNectar, int blueNectar, double distance) {
        int allyNectarCount = (ALLIANCE_COLOR == Alliance.RED) ? redNectar : blueNectar;
        return pollen + allyNectarCount + distance * distanceWeight;
    }
}
