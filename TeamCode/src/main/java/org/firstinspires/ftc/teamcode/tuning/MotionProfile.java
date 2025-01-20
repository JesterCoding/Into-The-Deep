package org.firstinspires.ftc.teamcode.tuning;

public class MotionProfile {
    private double accel;
    private double maxVelo;

    private double accelDistance;
    private double coastDistance;
    private double totalDist;

    public enum MotionState {
        ACCEL,
        COAST,
        DECCEL,
        IDLE
    }

    public MotionState motionState = MotionState.IDLE;

    public MotionProfile(double accel, double maxVelo) {
        this.accel = accel;
        this.maxVelo = maxVelo;
    }

    //capture start encoder velo
    public void calculateProfile(double dist) {
        totalDist = dist;
        accelDistance = Math.pow(maxVelo, 2) / (2 * accel);
        coastDistance = (2 * accelDistance > dist) ? dist - 2 * accelDistance : -1;
    }

    public void getProfileState(double currPos) {
        if (coastDistance == -1) {
            motionState = MotionState.IDLE;
        } else if (currPos < accelDistance) {
            motionState = MotionState.ACCEL;
        } else if (currPos < coastDistance) {
            motionState = MotionState.COAST;
        } else {
            motionState = MotionState.DECCEL;
        }
    }

    public double getTotalDist()
    {
        return totalDist;
    }
}