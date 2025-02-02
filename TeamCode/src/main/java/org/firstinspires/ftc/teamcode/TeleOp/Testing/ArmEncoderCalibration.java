package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp (group = "subsystems")
@Config
public class ArmEncoderCalibration extends LinearOpMode
{
    private RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init();
        waitForStart();

        while(opModeIsActive())
        {
            robot.telemetryUpdate();
        }
    }
}
