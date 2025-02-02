package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(group="subsystems")
public class IntakeTest extends LinearOpMode {

    Intake intake;
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(robot);

        waitForStart();

        while(opModeIsActive()) {

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

            telemetry.addLine("-------IntakeTest Commands-------");
            telemetry.addData("Intake", "X");
            telemetry.addData("Hold", "Y");
            telemetry.addData("Idle", "A");
            telemetry.addData("Release", "B");
            telemetry.update();
        }
    }
}
