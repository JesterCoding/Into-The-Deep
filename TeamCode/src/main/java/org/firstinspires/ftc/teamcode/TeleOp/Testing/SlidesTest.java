package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Config
@TeleOp(group = "subsystemstest")
public class SlidesTest extends LinearOpMode {
    Slides slides;
    PivotArm pivot;

    float setpoint = 0;
    float exSetpoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new Slides(hardwareMap);
        pivot = new PivotArm(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            slides.updatePos();
            pivot.updatePos();
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
                setpoint = 200;
            }

            slides.setTargetDist(exSetpoint);
            pivot.setTargetDist(setpoint);
            slides.runToPos();
            pivot.runToPos();


            telemetry.addData("setpoint", slides.targetHeight);
            telemetry.addData("pivot setpoint", pivot.targetHeight);
            telemetry.addData("pivot pos", pivot.getPosition());
            telemetry.addData("currPos", slides.getPosition());


            telemetry.update();
        }
    }
}