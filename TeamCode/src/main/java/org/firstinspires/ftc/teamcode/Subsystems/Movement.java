package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
    This class dictates the movement of the drivetrain
    and the **respective encoder readings** resulting from the movement
 */
public class Movement {

    private DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    RobotHardware robot = null;

    public Movement(RobotHardware hardwareMap)
    {
        robot = hardwareMap;
        robot.init();
        leftFrontDrive = hardwareMap.getLeftFrontDrive();
        leftBackDrive = hardwareMap.getLeftBackDrive();
        rightFrontDrive = hardwareMap.getRightFrontDrive();
        rightBackDrive = hardwareMap.getRightBackDrive();
    }



//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                        )
//                )
//        );
//
//        // Create an object to receive the IMU angles
//        YawPitchRollAngles robotOrientation;
//        robotOrientation = imu.getRobotYawPitchRollAngles();
//
//// Now use these simple methods to extract each angle
//// (Java type double) from the object you just created:
//        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
//        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

    // Initialize the hardware variables. Note that the strings used here must correspond
    // to the names assigned during the robot configuration step on the DS or RC devices.


    // ########################################################################################
    // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
    // ########################################################################################
    // Most robots need the motors on one side to be reversed to drive forward.
    // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
    // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
    // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
    // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
    // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
    // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

    /**
     *
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   Right/Left strafing power (-1.0 to 1.0) +ve
     * @param yaw
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine drive and turn for blended motion.

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial - lateral - yaw;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive both wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        robot.telemetryUpdate();
    }
    // Wait for the game to start (driver presses START)

    //
}

