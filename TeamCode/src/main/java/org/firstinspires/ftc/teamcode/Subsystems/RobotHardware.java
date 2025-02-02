package org.firstinspires.ftc.teamcode.Subsystems;

/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AprilTags;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    //Driving Motors for the drive train
    private DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;

    // Arm motors for slide and pivot of the arm
    // Pivot rotates the arm wrt to the movement plane
    // Slides extend the arm of the robot outwards
    private DcMotorEx topPivot, bottomPivot, topSlides, bottomSlides = null;

    //SwitchIntake "flips" the beltIntake up and down
    //rotIntake rotates the intake
    private Servo beltIntake, rotIntake, switchIntake = null;

    //AprilTag Detection Camera
    //Add another one here if we put another one on the robot
    private AprilTags webCam = new AprilTags();

    //ColorSensor to detect different types of blocks
    public ColorSensor color = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO =  0.5;
    public static final double HAND_SPEED =  0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER =  0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    public MecanumDrive drive = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()
    {

        //MecanumDrive drive = new MecanumDrive(myOpMode.hardwareMap, new Pose2d(0, 0, 0)); //Come back to check this

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        topPivot = myOpMode.hardwareMap.get(DcMotorEx.class, "topRightRotation");
        topPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topPivot.setDirection(DcMotorEx.Direction.REVERSE);

        bottomPivot = myOpMode.hardwareMap.get(DcMotorEx.class, "bottomRightRotation");
        bottomPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomPivot.setDirection(DcMotorEx.Direction.REVERSE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topSlides = myOpMode.hardwareMap.get(DcMotorEx.class, "topLeftRotation");
        topSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topSlides.setDirection(DcMotor.Direction.FORWARD);

        bottomSlides = myOpMode.hardwareMap.get(DcMotorEx.class, "bottomLeftRotation");
        bottomSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomSlides.setDirection(DcMotor.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        switchIntake = myOpMode.hardwareMap.get(Servo.class, "leftPivot");
        switchIntake.setDirection(Servo.Direction.REVERSE);
        beltIntake = myOpMode.hardwareMap.get(Servo.class, "intake");
        beltIntake.setDirection(Servo.Direction.REVERSE);
        rotIntake = myOpMode.hardwareMap.get(Servo.class, "rotation");
        //rotIntake.setPosition();
        //May be needed for setting a custom starting position

       // webCam.initAprilTag(myOpMode.hardwareMap, "Webcam 1");

       // color = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void telemetryUpdate()
    {
        myOpMode.telemetry.addData("Status", "Run Time: " + runtime.toString());
        myOpMode.telemetry.addLine("-------------Drive Train Values-----------------");
        myOpMode.telemetry.addLine("Robot Motor Power");
        myOpMode.telemetry.addData("FrontLeft/FrontRight/BackLeft/BackRight Power:",
                "%4.2f, %4.2f, %4.2f, %4.2f",
                leftFrontDrive.getPower(), rightFrontDrive.getPower(),
                leftBackDrive.getPower(), rightBackDrive.getPower());
/*
        myOpMode.telemetry.addLine("-------------IMU Values-----------------");
        Pose2d pose = drive.localizer.getPose();
        myOpMode.telemetry.addData("X-Position", pose.position.x);
        myOpMode.telemetry.addData("Y-Position", pose.position.y);
        myOpMode.telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        /*Add encoder and imu values here when you have time*/
        myOpMode.telemetry.addLine("-------------Arm Motor Values-----------------");
        myOpMode.telemetry.addLine("Pivot Values");
        myOpMode.telemetry.addData("Top/Bottom Pivot Power:","%4.2f, %4.2f",
                topPivot.getPower(), bottomPivot.getPower());
        myOpMode.telemetry.addData("Pivot Position:",
                bottomPivot.getCurrentPosition());
        myOpMode.telemetry.addLine("Slide Values");
        myOpMode.telemetry.addData("Top/Bottom Slide Power:","%4.2f, %4.2f",
                topSlides.getPower(), bottomSlides.getPower());
        myOpMode.telemetry.addData("Slide Position:", bottomSlides.getCurrentPosition());
        myOpMode.telemetry.addLine("-------------Intake Values-----------------");
        myOpMode.telemetry.addData("leftPivot/release/rotation", "%4.2f, %4.2f, %4.2f",
                switchIntake.getPosition(), beltIntake.getPosition(), rotIntake.getPosition());

        myOpMode.telemetry.update();
    }


    public DcMotorEx getLeftFrontDrive()
    {
        return leftFrontDrive;
    }
    public DcMotorEx getRightFrontDrive()
    {
        return rightFrontDrive;
    }
    public DcMotorEx getLeftBackDrive()
    {
        return leftBackDrive;
    }
    public DcMotorEx getRightBackDrive()
    {
        return rightBackDrive;
    }

    public DcMotorEx getTopPivot()
    {
        return topPivot;
    }

    public DcMotorEx getBottomPivot()
    {
        return bottomPivot;
    }

    public DcMotorEx getBottomSlides()
    {
        return bottomSlides;
    }

    public DcMotorEx getTopSlides()
    {
        return topSlides;
    }

    public Servo getBeltIntake()
    {
        return beltIntake;
    }

    public Servo getRotIntake()
    {
        return rotIntake;
    }

    public Servo getSwitchIntake()
    {
        return switchIntake;
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    /*
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        leftFrontPower.setPower(leftWheel);

        rightDrive.setPower(rightWheel);
    }


    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)

    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset

    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
    */
}
