package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

@Config
public class Slides {

    DcMotorEx topSlides, bottomSlides;

    //DO NOT TOUCH!!!
    public static double Kp = 0.0080;
    public static double Ki = 0.0008;
    public static double Kd = 0.0025;

    public static double k = 1; //Steepness of Sigmoid Graph

    public double errorSumPositional;
    double prevErrorPositional;
    double errorSumVelocity;
    double prevErrorVelocity;

    public double currHeight = 0;
    public double targetHeight = 0;

    private double currPower = 0;

    private int maxHeight = 700;
    private int minHeight = 0;

    private double modifier = 1;

    private RobotHardware robot = null;

    public Slides(RobotHardware hardwareMap) {
        robot = hardwareMap;
        robot.init();

        topSlides = hardwareMap.getTopSlides();
        bottomSlides = hardwareMap.getBottomSlides();

        //persistent no bueno
        errorSumPositional = 0;
        errorSumVelocity = 0;
        prevErrorVelocity = 0;
        prevErrorPositional = 0;
    }

    /*
    700 for Top Basket
    219 for Slides 42 inches
    */

    public void runToPos()
    {
        double error = targetHeight - currHeight;

        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;  // Reset the integral term when the error changes sign
        }
        if (Math.abs(errorSumPositional) < 200)
            errorSumPositional += error;                           // Accumulate error for integral

        double errorDiff = error - prevErrorPositional; // Compute error difference for derivative
        double pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;
        prevErrorPositional = error;

        //modifier = sigmoid(pidOutput);

        topSlides.setPower(pidOutput*modifier); //
        bottomSlides.setPower(pidOutput*modifier); //

        updatePos();
        robot.telemetryUpdate();
    }

    private double sigmoid(double x)
    {
        // Adjust steepness of the sigmoid curve (higher values = steeper transition)
        // return Math.abs(1 / (1 + Math.exp(-k * x))-0.5)+0.5; // Scaled to range [0, 1]
        return 2*Math.abs(1 / (1 + Math.exp(-k * x))-0.5);
    }

    public void stabilize() {
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
        double pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;

        // Apply sigmoid scaling to smooth the output
        // double scaledOutput = sigmoid(error);

        // Update the previous error for derivative calculation
        prevErrorPositional = error;

        currPower = pidOutput;
        // Set motor powers with scaled output
        setCustomPower(currPower);
        robot.telemetryUpdate();
    }

    public void setTargetDist(double dist) {
        targetHeight = dist;
    }



    public void setCurrPower(double power)
    {
        if (currHeight > maxHeight || currHeight < minHeight)
        {
            power = Math.min(0, power);
        }
        topSlides.setPower(power);
        bottomSlides.setPower(power);
        updatePos();
    }

    public void setCustomPower(double pow) {
        topSlides.setPower(pow);
        bottomSlides.setPower(pow);
    }

    public double getCurrPower()
    {
        return currPower;
    }

    public void updatePos() {
        currHeight = getPosition();
    }

    public int getPosition() {
        return bottomSlides.getCurrentPosition();
    }
}
