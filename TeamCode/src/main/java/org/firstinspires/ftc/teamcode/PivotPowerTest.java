package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Disabled
@Config
@TeleOp(name = "PivotPowerTest", group = "subsystemstest")
public class PivotPowerTest extends LinearOpMode
{
    public double currPower = 0;
    public int currPos, targetPos = 0;

    public static double change = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotHardware robot = new RobotHardware(this);
        PivotArm pivot = new PivotArm(robot);

        waitForStart();

        telemetry.addData("CurrPos", currPos);
        telemetry.addData("TargetPos", targetPos);
        telemetry.addData("CurrPower", currPower);
        telemetry.update();

        while(opModeIsActive())
        {
            while (currPos < 200)
            {
                currPos = pivot.getPosition();
                targetPos = currPos+1;
                currPower+=change;
                pivot.setCustomPower(currPower);
                pivot.updatePos();
                currPos = pivot.getPosition();
            }

            telemetry.addData("CurrPos", currPos);
            telemetry.addData("TargetPos", targetPos);
            telemetry.addData("CurrPower", currPower);
            telemetry.update();
        }

    }

}
