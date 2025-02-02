package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
@Config
public class ArmPid extends LinearOpMode
{
    RobotHardware robot = new RobotHardware(this);
    PivotArm pivot;
    Slides slide;

    float pivotSetPoint, slideSetPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        pivot = new PivotArm(robot);
        slide = new Slides(robot);
        waitForStart();
        while(opModeIsActive())
        {
            if (gamepad1.x)
            {
                pivotSetPoint = 30;
                slideSetPoint = 574;
            }

            pivot.setTargetDist(pivotSetPoint);
            slide.setTargetDist(slideSetPoint);

            pivot.runToPos();
            slide.runToPos();

            pivot.updatePos();
            slide.updatePos();

            robot.telemetryUpdate();
        }
    }
}
