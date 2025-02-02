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
public class PivotPID extends LinearOpMode
{
    PivotArm pivot;

    int setpoint = 0;

    //0, 180, less 90, more 90, Around 20 deg
    public static int nintyDeg = 360;
    public static int zero = 0;
    // PickUp Walled Specimen(30, 574)
    //(329, 669)
    //(426, 27) (Pivot, Slide) positions
    //(402, 27)
    //(402, -20)
    //(770, -27) //Change Slide Position to 219

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivot = new PivotArm(robot);

        while (opModeIsActive())
        {

            if (gamepad1.x) {
                setpoint += 5;
                setpoint = Math.min(setpoint, 600);
            }
            if (gamepad1.b) {
                setpoint -= 5;
                setpoint = Math.max(setpoint, 0);
            }

            if (gamepad1.right_bumper) {
                setpoint = nintyDeg;
            }


            pivot.setTargetDist(setpoint);

            pivot.runToPos();

            //if (Math.abs(pivot.targetHeight - pivot.currHeight) < 2)
            //{
              //  pivot.stabilize();
              //  pivot.updatePos();
            //}

            pivot.updatePos();

            telemetry.addData("PivotTarget", pivot.targetHeight);
            telemetry.addData("PivotCurrPos", pivot.getPosition());
            telemetry.addData("PivotPower", pivot.getCurrPower());

            robot.telemetryUpdate();
        }
    }
}
