package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Config
@TeleOp(group = "Subsystems")
public class TeleOpTestV1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Slides slides;
    PivotArm pivot;
    Movement movement;

    float setpoint = 0;
    float exSetpoint = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        movement = new Movement(hardwareMap);
        slides = new Slides(hardwareMap);
        pivot = new PivotArm(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            slides.updatePos();
            pivot.updatePos();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            if(gamepad1.right_bumper)
            {
                setpoint = 200;
            }

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

            movement.move(axial, lateral, yaw);

            slides.setTargetDist(exSetpoint);
            pivot.setTargetDist(setpoint);
            slides.runToPos();
            pivot.runToPos();
        }
    }
}
