package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "CustomPivotTest", group = "subsystemstest")
public class CustomPivotTest extends LinearOpMode
{
    DcMotorEx bottomPivot, topPivot;

    double customPower = 1.0;



    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        bottomPivot = hardwareMap.get(DcMotorEx.class, "topRightRotation");
        bottomPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        topPivot = hardwareMap.get(DcMotorEx.class, "bottomRightRotation");
        topPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive())
        {
            topPivot.setPower(customPower);
            bottomPivot.setPower(customPower);
            telemetry.addData("Power", customPower);

            telemetry.update();
        }

    }
}
