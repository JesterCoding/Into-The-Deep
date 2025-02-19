package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.lintuPID;

@Disabled
@Config
@TeleOp(name = "ArmTestPID", group = "subsystems")
public class SlidesTest extends LinearOpMode {
    Slides slides;
    PivotArm pivot;

    int setpoint = 0;
    int exSetpoint = 0;

    public static int nintyDeg = 439;

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = new Slides(robot);

        pivot = new PivotArm(robot);

        while (opModeIsActive())
        {
            slides.updatePos();

            if (gamepad1.x) {
                setpoint += 5;
            }
            if (gamepad1.b) {
                setpoint -= 5;
                setpoint = Math.max(setpoint, 0);

            }

            if (gamepad1.y) {
                exSetpoint += 5;
            }
            if (gamepad1.a) {
                exSetpoint -= 5;
                exSetpoint = Math.max(exSetpoint, 0);
            }

            if (gamepad1.right_bumper) {
                setpoint = nintyDeg;
            }

            slides.setTargetDist(exSetpoint);
            pivot.setTargetDist(setpoint);
           // slides.runToPos(exSetpoint);

            pivot.runToPos();

            if (Math.abs(pivot.targetHeight - pivot.currHeight) < 2)
            {
                pivot.stabilize();
                pivot.updatePos();
            }

            pivot.updatePos();

            telemetry.addData("PivotTarget", pivot.targetHeight);
            telemetry.addData("PivotCurrPos", pivot.getPosition());
            telemetry.addData("SlidesCurr", slides.getPosition());
            telemetry.addData("SlidesTarget", slides.targetHeight);
            telemetry.addData("SlidePower", slides.getCurrPower());
            telemetry.addData("PivotPower", pivot.getCurrPower());

            robot.telemetryUpdate();
        }
    }
}