package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Config
@TeleOp(name = "CustomPowerOnPivot")
public class PivotMotorTest extends LinearOpMode
{
    DcMotorEx topPivot, bottomPivot, topSlides, bottomSlides;

    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        topPivot = robot.getTopPivot();
        bottomPivot = robot.getBottomPivot();
        topSlides = robot.getTopSlides();
        bottomSlides = robot.getBottomSlides();

        waitForStart();
        while(opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            if (gamepad1.dpad_down) //should push down
            {
                bottomPivot.setPower(power);
                topPivot.setPower(-1*power);
            }

            if (gamepad1.dpad_up) //should push up
            {
                topPivot.setPower(power);
                bottomPivot.setPower(-1*power);
            }
            if (gamepad1.dpad_left)
            {
                topSlides.setPower(power);
                bottomSlides.setPower(power);
            }
            if (gamepad1.dpad_right)
            {
                topSlides.setPower(-1*power);
                bottomSlides.setPower(-1*power);
            }

            telemetry.addData("bottomPivot Pos", bottomPivot.getCurrentPosition());
            telemetry.addData("topPivot Pos", topPivot.getCurrentPosition());

        }
    }
}
