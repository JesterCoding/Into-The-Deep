package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "ArmRotTesting", group = "Linear OpMode")
public class ArmRotTesting extends LinearOpMode
{
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor topRightRotation = null;
        private DcMotor bottomRightRotation = null;

        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            // topRightRotation  = hardwareMap.get(DcMotor.class, "topRightRotation");
            // bottomRightRotation  = hardwareMap.get(DcMotor.class, "bottomRightRotation");
            topRightRotation = hardwareMap.get(DcMotor.class, "topRightRotation");
            bottomRightRotation = hardwareMap.get(DcMotor.class, "bottomRightRotation");

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
            //  topRightRotation.setDirection(DcMotor.Direction.FORWARD);
            //  bottomRightRotation.setDirection(DcMotor.Direction.FORWARD);

            topRightRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomRightRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            topRightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottomRightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            topRightRotation.setDirection(DcMotor.Direction.REVERSE);
            bottomRightRotation.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
               double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry
                double topExtensionPower = axial;
                double bottomExtensionPower = axial;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(topExtensionPower), Math.abs(bottomExtensionPower));
                //int topPosition = Math.max(topRightRotation.getCurrentPosition(), bottomRightRotation.getCurrentPosition());


                if (max > 1.0) {
                    topExtensionPower /= max;
                    bottomExtensionPower /= max;
                }

//                if (topPosition > 680)
//                {
//                    topExtensionPower = Math.min(topExtensionPower, 0);
//                    bottomExtensionPower = Math.min(bottomExtensionPower, 0);
//                }


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
                //   topRightRotation.setPower(leftFrontPower);
                //   bottomRightRotation.setPower(rightFrontPower);
                topRightRotation.setPower(topExtensionPower);
                bottomRightRotation.setPower(bottomExtensionPower);




                //topRightRotation.setTargetPosition(topRightRotation.getCurrentPosition());
                //topRightRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //topRightRotation.setPower(1.0);

                //bottomRightRotation.setTargetPosition(topRightRotation.getCurrentPosition());
                //bottomRightRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //bottomRightRotation.setPower(1.0);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Power Top/Bottom", "%4.2f, %4.2f", topExtensionPower, bottomExtensionPower);
                telemetry.addData("Top Encoder Data:", topRightRotation.getCurrentPosition());
                telemetry.addData("Bottom Encoder Data:", bottomRightRotation.getCurrentPosition());
                telemetry.update();
            }
        }
    }
