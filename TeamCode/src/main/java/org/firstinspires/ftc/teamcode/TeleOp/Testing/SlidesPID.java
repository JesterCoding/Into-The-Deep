package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Config
@TeleOp(group = "subsystems")
public class SlidesPID extends LinearOpMode
{
    Slides slide;

    int setpoint = 0;

    public static int nintyDeg = 360;

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide = new Slides(robot);

        while (opModeIsActive())
        {

            if (gamepad1.x) {
                setpoint += 5;
                setpoint = Math.min(setpoint, 700);
            }
            if (gamepad1.b) {
                setpoint -= 5;
                setpoint = Math.max(setpoint, 0);
            }

            if (gamepad1.right_bumper) {
                setpoint = nintyDeg;
            }


            slide.setTargetDist(setpoint);

            slide.runToPos();

            if (Math.abs(slide.targetHeight - slide.currHeight) < 2)
            {
                slide.stabilize();
                slide.updatePos();
            }

            slide.updatePos();

            telemetry.addData("PivotTarget", slide.targetHeight);
            telemetry.addData("PivotCurrPos", slide.getPosition());
            telemetry.addData("PivotPower", slide.getCurrPower());

            robot.telemetryUpdate();
        }
    }
}
