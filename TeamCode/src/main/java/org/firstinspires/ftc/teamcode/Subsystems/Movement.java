package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement {

    private DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    public Movement(HardwareMap hardwareMap)
    {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "backRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
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

    public void move(double axial, double lateral, double yaw)
    {
        double max;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial - lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
    }
    // Wait for the game to start (driver presses START)

    //
}

