package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(group = "subsystems")
@Config
public class IndividualMotorTest extends LinearOpMode
{
    private RobotHardware robot = new RobotHardware(this);
    private static double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init();
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x) //FrontLeft motor
            {
                if(robot.getLeftFrontDrive().getPower() == power)
                {
                    robot.getLeftFrontDrive().setPower(0);
                }
                else
                {
                    robot.getLeftFrontDrive().setPower(power);
                }
            }
            if(gamepad1.y) //BackLeft motor
            {
                if(robot.getLeftBackDrive().getPower() == power)
                {
                    robot.getLeftBackDrive().setPower(0);
                }
                else
                {
                    robot.getLeftBackDrive().setPower(power);
                }
            }
            if(gamepad1.a) //FrontRight motor
            {
                if(robot.getRightFrontDrive().getPower() == power)
                {
                    robot.getRightFrontDrive().setPower(0);
                }
                else
                {
                    robot.getRightFrontDrive().setPower(power);
                }
            }
            if(gamepad1.b) //BackRight motor
            {
                if(robot.getRightBackDrive().getPower() == power)
                {
                    robot.getRightBackDrive().setPower(0);
                }
                else
                {
                    robot.getRightBackDrive().setPower(power);
                }
            }
            if(gamepad1.dpad_up) //TopSlide Motor
            {
                if(robot.getTopSlides().getPower() == power)
                {
                    robot.getTopSlides().setPower(0);
                }
                else
                {
                    robot.getTopSlides().setPower(power);
                }
            }
            if(gamepad1.dpad_down) //BottomSlide Motor
            {
                if(robot.getBottomSlides().getPower() == power)
                {
                    robot.getBottomSlides().setPower(0);
                }
                else
                {
                    robot.getBottomSlides().setPower(power);
                }
            }
            if(gamepad1.dpad_left) //TopPivot Motor
            {
                if(robot.getTopPivot().getPower() == power)
                {
                    robot.getTopPivot().setPower(0);
                }
                else
                {
                    robot.getTopPivot().setPower(power);
                }
            }
            if(gamepad1.dpad_right)
            {
                if(robot.getBottomPivot().getPower() == power)
                {
                    robot.getBottomPivot().setPower(0);
                }
                else
                {
                    robot.getBottomPivot().setPower(power);
                }
            }

            telemetry.addData("Front/Back Left Drive Motor:", "X,Y");
            telemetry.addData("Front/Back Right Drive Motor:", "A,B");
            telemetry.addData("Top/Bottom Pivot:", "<-, ->");
            telemetry.addData("Top/Bottom Slides:", "dpadTop, dpadBottom");
            telemetry.update();
        }

    }

}
