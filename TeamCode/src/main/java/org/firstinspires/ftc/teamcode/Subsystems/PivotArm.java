package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Arrays;

@Config
public class PivotArm {

    private static class Filters {
        double alpha;
        double[] channels;

        public Filters(int channels, double alpha) {
            this.alpha = alpha;
            this.channels = new double[channels];
            Arrays.fill(this.channels, 1);
        }

        //exponential moving averages filter
        public double[] emaFilter(double[] val) {
            for (int i = 0; i < channels.length; i++) {
                channels[i] = (alpha * val[i] + (1 - alpha) * channels[i]);
            }
            return channels;
        }
    }


    DcMotorEx bottomPivot;
    DcMotorEx topPivot;
    DcMotorEx topEx;
    DcMotorEx bottomEx;

    public double currHeight = 0;
    public double targetHeight = 0;


    public double errorSumPositional;
    double prevErrorPositional;
    double errorSumVelocity;
    double prevErrorVelocity;

    double pidOutput;

    //DO NOT TOUCH!!!
    public static double Kp = 0.035;
    public static double Ki = 0.0008;
    public static double Kd = 0.025;


    Filters filter;

    public PivotArm(HardwareMap hardwareMap) {


        bottomPivot = hardwareMap.get(DcMotorEx.class, "topRightRotation");
        bottomPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        topPivot = hardwareMap.get(DcMotorEx.class, "bottomRightRotation");
        topPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        filter = new Filters(1, 0.02);

        //persistent no bueno
        errorSumPositional = 0;
        errorSumVelocity = 0;
        prevErrorVelocity = 0;
        prevErrorPositional = 0;
    }


    //DIST IN TICKS
    //normalize dist, but i dont care
    public void runToPos() {
        double modifier = 1;
        if (targetHeight < currHeight) {
            modifier = 0.9;
        }

        // Error calculation
        double error = targetHeight - currHeight;

        // Reset integral term when error changes sign
        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;
        }

        // Accumulate error for integral term (limit to prevent wind-up)
        if (Math.abs(errorSumPositional) < 200) {
            errorSumPositional += error;
        }

        // Compute derivative term
        double errorDiff = error - prevErrorPositional;

        // Compute raw PID output
        pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;

        // Apply sigmoid scaling to smooth the output
        //double scaledOutput = sigmoid(pidOutput);

        //pidOutput = sigmoid(pidOutput);
        // Update the previous error for derivative calculation
        prevErrorPositional = error;

        // Set motor powers with scaled output
        bottomPivot.setPower(/*scaledOutput * */ pidOutput*modifier);
        topPivot.setPower(/*scaledOutput * */ pidOutput*modifier);
    }

    // Sigmoid function for scaling
    private double sigmoid(double x) {
        double k = 5.0; // Adjust steepness of the sigmoid curve (higher values = steeper transition)
        return 1 / (1 + Math.exp(-k * x)) - 0.5; // Scaled to range [-0.5, 0.5]
    }


    public void setTargetDist(double dist) {
        targetHeight = dist;
    }

    public void setCustomPower(double pow) {
        bottomPivot.setPower(pow);
        topPivot.setPower(pow);
    }



    public void updatePos() {
        currHeight = getPosition();
    }

    public double getPosition() {
        return bottomPivot.getCurrentPosition();
    }

    public double getVelocity() {
        return bottomPivot.getVelocity();
    }
}