package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Config
@TeleOp
public class EncoderToPivotMotorTest extends LinearOpMode {

    public DcMotorEx topPivot, bottomPivot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        topPivot = robot.getTopPivot();
        bottomPivot = robot.getBottomPivot();

        waitForStart();

        while(opModeIsActive())
        {
            //topPivot.setPower(customPower);
            // bottomPivot.setPower(customPower);
            double topPos = topPivot.getCurrentPosition();
            double bottomPos = bottomPivot.getCurrentPosition();
            double power = -gamepad1.left_stick_y;

            topPivot.setPower(power);
            bottomPivot.setPower(-1*power);

            telemetry.addData("Encoder Top", topPos);
            telemetry.addData("Encoder Bottom", bottomPos);
            telemetry.addData("Power Top/Bottom", power);
            telemetry.update();
        }
    }

}
