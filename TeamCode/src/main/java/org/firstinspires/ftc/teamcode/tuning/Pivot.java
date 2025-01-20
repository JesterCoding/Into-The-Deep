package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.MotionProfile;

import java.util.Arrays;


/*
Note: This file is directly obtained from Future's Centerstage robot config's.
This will NOT necessarily work with your robot desgin. Use with caution.
(Prolly will have to create your own custom version of this depending on your robot design)
 */

@Config
public class Pivot {

    private static class Filters {
        double alpha;
        double[] channels;

        //Initialization of Constructor
        //Filling some array with all 1's
        public Filters(int channels, double alpha) {
            this.alpha = alpha;
            this.channels = new double[channels];
            Arrays.fill(this.channels, 1);
        }

        //exponential moving averages filter
        //Pass in some wieghts that will change channel values
        public double[] emaFilter(double[] val) {
            for (int i = 0; i < channels.length; i++) {
                channels[i] = (alpha * val[i] + (1 - alpha) * channels[i]);
            }
            return channels;
        }
    }


    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    public static double accel = 10;
    MotionProfile motionProfile;
    double currDist;
    public double currHeight = 0;
    public double targetHeight = 0;

    public double MAX_VELO = 800;

    public double errorSumPositional;
    double prevErrorPositional;
    double errorSumVelocity;
    double prevErrorVelocity;

    //DO NOT TOUCH!!!
    public static double Kp = 0.025;
    public static double Ki = 0.0008;
    public static double Kd = 0.025;

    public final static double VKp = 0.00045;
    public final static double VKi = 0.000085;
    public final static double VKd = 0.001;
    public final static double VKf = 0.001;

    Filters filter;

    public Pivot(HardwareMap hardwareMap) {

        //Change the name of the device here
        leftSlide = hardwareMap.get(DcMotorEx.class, "topLeftRotation");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Change the name of this device here?!
        rightSlide = hardwareMap.get(DcMotorEx.class, "bottomLeftRotation");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        filter = new Filters(1, 0.02);

        motionProfile = new MotionProfile(accel, MAX_VELO);
        motionProfile.motionState = MotionProfile.MotionState.IDLE;

        //persistent no bueno
        errorSumPositional = 0;
        errorSumVelocity = 0;
        prevErrorVelocity = 0;
        prevErrorPositional = 0;
    }

    //USE ONLY FOR MOTION PROFILING
    public double runToVelo(double velo) {

        double error = velo - filter.emaFilter(new double[]{leftSlide.getVelocity()})[0];

        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;  // Reset the integral term when the error changes sign
        }

        if (Math.abs(errorSumVelocity) < 50)
            errorSumVelocity += error;                           // Accumulate error for integral
        double errorDiff = error - prevErrorVelocity; // Compute error difference for derivative
        double pidOutput = VKp * error + VKi * errorSumVelocity + VKd * errorDiff + VKf * error;
        prevErrorVelocity = error;

        leftSlide.setPower(pidOutput);
        rightSlide.setPower(pidOutput);
        return error;
    }

    public void createMotionProfile(double dist) {
        currDist = dist;
        motionProfile.calculateProfile(dist);
    }

    public void runWithMotionProfile(double dist) {
        motionProfile.getProfileState(dist);
        switch (motionProfile.motionState) {
            case IDLE:
                runToPos(targetHeight, currHeight);
                break;
            case ACCEL:
                runToVelo(leftSlide.getVelocity() + accel);
                break;
            case COAST:
                runToVelo(MAX_VELO);
                break;
            case DECCEL:
                if (Math.abs(leftSlide.getVelocity()) < 1) {
                    motionProfile.motionState = MotionProfile.MotionState.IDLE;
                }
                runToVelo(leftSlide.getVelocity() - accel);
                break;
        }
    }


    //DIST IN TICKS
    //normalize dist, but i dont care
    public void runToPos(double target, double current) {
        double modifier = 1;
        if (target < current) {
            modifier = 0.9;
        }
        double error = target - current;

        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;  // Reset the integral term when the error changes sign
        }
        if (Math.abs(errorSumPositional) < 200)
            errorSumPositional += error;                           // Accumulate error for integral
        double errorDiff = error - prevErrorPositional; // Compute error difference for derivative
        double pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;
        prevErrorPositional = error;


        leftSlide.setPower(pidOutput*modifier); //
        rightSlide.setPower(pidOutput*modifier); //
    }

    public void setTargetDist(double dist) {
        targetHeight = dist;
    }

    public void setCustomPower(double pow) {
        leftSlide.setPower(pow);
        rightSlide.setPower(pow);
    }

    public void updatePos() {
        currHeight = getPosition();
    }

    public double getPosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition())/2.0;
    }

    public double getVelocity() {
        return leftSlide.getVelocity();
    }
}