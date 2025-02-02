package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Disabled
@Config
@TeleOp(group = "subsystems")
public class TeleOpTestV1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Slides slides;
    PivotArm pivot;
    Movement movement;

    int setpoint = 0;
    int exSetpoint = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        movement = new Movement(robot);
        slides = new Slides(robot);
        pivot = new PivotArm(robot);

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

            movement.driveRobot(axial, lateral, yaw);

            slides.setTargetDist(setpoint);
            slides.runToPos();

            telemetry.addData("SlidesCurr", slides.getPosition());
            telemetry.addData("SlidesTarget", slides.targetHeight);
            telemetry.addData("SlidePower", slides.getCurrPower());

            telemetry.update();
        }
    }
}
