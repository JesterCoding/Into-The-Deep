package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Slides {

    DcMotorEx topEx;
    DcMotorEx bottomEx;

    //DO NOT TOUCH!!!
    public static double Kp = 0.025;
    public static double Ki = 0.0008;
    public static double Kd = 0.025;

    public double errorSumPositional;
    double prevErrorPositional;
    double errorSumVelocity;
    double prevErrorVelocity;

    public double currHeight = 0;
    public double targetHeight = 0;



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

    Filters filter;

    public Slides(HardwareMap hardwareMap) {
        topEx = hardwareMap.get(DcMotorEx.class, "topLeftRotation");
        topEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomEx = hardwareMap.get(DcMotorEx.class, "bottomLeftRotation");
        bottomEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        filter = new Filters(1, 0.02);

        //persistent no bueno
        errorSumPositional = 0;
        errorSumVelocity = 0;
        prevErrorVelocity = 0;
        prevErrorPositional = 0;
    }

    public void runToPos() {
        double modifier = 1;
        if (targetHeight < currHeight) {
            modifier = 0.9;
        }
        double error = targetHeight - currHeight;

        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;  // Reset the integral term when the error changes sign
        }
        if (Math.abs(errorSumPositional) < 200)
            errorSumPositional += error;                           // Accumulate error for integral
        double errorDiff = error - prevErrorPositional; // Compute error difference for derivative
        double pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;
        prevErrorPositional = error;


        topEx.setPower(pidOutput*modifier); //
        bottomEx.setPower(pidOutput*modifier); //
    }


    public void setTargetDist(double dist) {
        targetHeight = dist;
    }

    public void setCustomPower(double pow) {
        topEx.setPower(pow);
        bottomEx.setPower(pow);
    }



    public void updatePos() {
        currHeight = getPosition();
    }

    public double getPosition() {
        return topEx.getCurrentPosition();
    }
}
