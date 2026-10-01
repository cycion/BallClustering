package org.firstinspires.ftc.teamcode.algorithm;

/**
 * Scorer is a functional interface to handle scoring algorithms. Nothing much, mostly just an
 * template
 */

// TODO: Update function signature
@FunctionalInterface
public interface Scorer {
    public double score(int pollen, int redNectar, int blueNectar, double distance);
}
