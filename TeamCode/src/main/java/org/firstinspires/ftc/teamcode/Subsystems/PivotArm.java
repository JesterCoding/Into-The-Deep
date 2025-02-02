package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Arrays;

@Config
public class PivotArm {

    DcMotorEx topPivot, bottomPivot;

    public double currHeight, targetHeight, currPower = 0;

    public double errorSumPositional,prevErrorPositional, errorSumVelocity, prevErrorVelocity;

    //DO NOT TOUCH!!!
    public static double Kp = 0.0045;      //0.035;
    public static double Ki = 0.0005;      //0.0008;
    public static double Kd = 0.015;      //0.025;

    public static double k = 0.013; //Sharpness of Sigmoid Curve

    //public static double resist = 0.;

    public double scaledOutput = 1;


    private RobotHardware robot = null;

    public PivotArm(RobotHardware hardwareMap)
    {
        robot = hardwareMap;
        robot.init();

        topPivot = hardwareMap.getTopPivot();
        bottomPivot = hardwareMap.getBottomPivot();

        //persistent no bueno
        errorSumPositional = 0;
        errorSumVelocity = 0;
        prevErrorVelocity = 0;
        prevErrorPositional = 0;
    }


    //DIST IN TICKS
    //normalize dist, but i dont care



    public void runToPos() {
        // Error calculation
        double error = targetHeight - currHeight;

        // Reset integral term when error changes sign
        if (error * prevErrorPositional <= 0) {
            errorSumPositional = 0;
        }

        // Accumulate error for integral term (limit to prevent wind-up)
        //wtvr we set as 90 should replace 360
        errorSumPositional = Math.min(errorSumPositional + error, targetHeight);


        // Compute derivative term
        double errorDiff = error - prevErrorPositional;

        // Compute raw PID output
        double pidOutput = Kp * error + Ki * errorSumPositional + Kd * errorDiff;

        // Apply sigmoid scaling to smooth the output
       // scaledOutput = sigmoid(error);

        // Update the previous error for derivative calculation
        prevErrorPositional = error;

        currPower = scaledOutput*pidOutput;
        // Set motor powers with scaled output
        setCustomPower(currPower);
        robot.telemetryUpdate();
    }

    private double sigmoid(double x)
    {
        // Adjust steepness of the sigmoid curve (higher values = steeper transition)
        // return Math.abs(1 / (1 + Math.exp(-k * x))-0.5)+0.5; // Scaled to range [0, 1]
        return 2*Math.abs(1 / (1 + Math.exp(-k * x))-0.5);
    }

    public void stabilize()
    {

    }


    public void setTargetDist(double dist) {
        targetHeight = dist;
    }

    public void setCustomPower(double pow) {
        topPivot.setPower(pow);
        bottomPivot.setPower(-1*pow);
    }

    public double getCurrPower()
    {
        return currPower;
    }

    public void updatePos() {
        currHeight = getPosition();
    }

    public int getPosition() {
        return bottomPivot.getCurrentPosition(); //topPivot encoder and bottomPivot encoder are flipped
    }
}