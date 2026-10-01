package org.firstinspires.ftc.teamcode.algorithm;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.structure.Cluster;

/**
 * Scorer is a functional interface to handle scoring algorithms. Nothing much, mostly just a
 * template
 */

// TODO: Update function signature
@FunctionalInterface
public interface Scorer {
    double score(Cluster cluster, Pose robotPose, double turretAngle);
}
