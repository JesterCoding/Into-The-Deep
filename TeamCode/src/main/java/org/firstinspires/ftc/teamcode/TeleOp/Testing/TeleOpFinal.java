package org.firstinspires.ftc.teamcode.TeleOp.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
public class TeleOpFinal extends LinearOpMode
{
    RobotHardware robot = new RobotHardware(this);

    private double axial, lateral, yaw = 0;
    private float pivotSetPoint = 0;

    PivotArm pivot;
    Slides slide;
    Intake intake;
    Movement driveTrain;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init();
        pivot = new PivotArm(robot);
        slide = new Slides(robot);
        intake = new Intake(robot);
        driveTrain = new Movement(robot);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addLine("Ready to Go...");

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveTrain.driveRobot(axial, lateral, yaw);

            if (gamepad1.dpad_up)
            {
                pivotSetPoint = 360; //In Ticks
            }
            if (gamepad1.dpad_down)
            {
                pivotSetPoint = 0; //In Ticks
            }
            pivot.setTargetDist(pivotSetPoint);
            pivot.runToPos();
            pivot.updatePos();

            if (gamepad1.x) {
                intake.intake();
            }

            if (gamepad1.y) {
                intake.hold();
            }

            if (gamepad1.a) {
                intake.idle();
            }

            if (gamepad1.b) {
                intake.release();
            }

            //Add other operations here


            telemetry.addLine("---------Directions For TeleOp------------");
            telemetry.addData("Forward/Backward DriveTrain", "LeftStick Up/Down");
            telemetry.addData("Left/Right DriveTrain", "LeftStick Left/Right");
            telemetry.addData("Rotate Left/Right", "RightStick Left/Right");
            telemetry.addData("Pivot Angle Up/Down", "dpad_up/dpad_down");
            telemetry.addData("Intake/Hold/Idle/Release", "X/Y/A/B");
            telemetry.update();

        }

    }
}
