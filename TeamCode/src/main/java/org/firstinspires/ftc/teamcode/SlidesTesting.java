package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
@Config
public class SlidesTesting extends LinearOpMode
{
    DcMotorEx topSlides, bottomSlides;
    public static double customPower = 0.8;
    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        Slides slide = new Slides(robot);
        waitForStart();

        while(opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            if (gamepad1.dpad_down)
            {
                float power = -1*gamepad1.right_stick_y; //Forward is positive power
                power = Math.min(1, power);
                power = Math.max(-1, power);
                slide.setCurrPower(power);
            }

            if (gamepad1.dpad_left)
            {
                slide.setCurrPower(customPower);
            }

            robot.telemetryUpdate();

        }
    }
}
