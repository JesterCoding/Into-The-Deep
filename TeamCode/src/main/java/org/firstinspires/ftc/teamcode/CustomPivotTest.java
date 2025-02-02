package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Config
@TeleOp(name = "CustomPivotTest", group = "subsystemstest")
public class CustomPivotTest extends LinearOpMode
{
    DcMotorEx bottomPivot, topPivot;

    public static double customPower = 0.107;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        PivotArm pivot = new PivotArm(robot);

        waitForStart();

        while(opModeIsActive())
        {
            pivot.setCustomPower(customPower);
           robot.telemetryUpdate();
        }
    }
}
