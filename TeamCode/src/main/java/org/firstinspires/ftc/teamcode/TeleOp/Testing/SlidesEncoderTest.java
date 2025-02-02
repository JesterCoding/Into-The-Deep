package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
@Config
public class SlidesEncoderTest extends LinearOpMode
{
    RobotHardware robot = new RobotHardware(this);

    public static double customPower = 0.1;

    public double power = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init();
        Slides slide = new Slides(robot);
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.x)
            {
                power = 0;
            }
            if (gamepad1.y)
            {
                power = customPower;
            }
            if (gamepad1.a)
            {
                power = -1*customPower;
            }

            slide.setCurrPower(power);

            telemetry.addLine("TeleOp Directions");
            telemetry.addData("Slides Extension", "Y");
            telemetry.addData("Slides De-extension", "A");
            telemetry.addData("Slides Stop", "X");
            telemetry.update();

            robot.telemetryUpdate();
        }
    }
}
