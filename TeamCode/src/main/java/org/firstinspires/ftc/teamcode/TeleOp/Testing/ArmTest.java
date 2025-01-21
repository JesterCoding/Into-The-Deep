package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(group="subsystemstest")
public class ArmTest extends LinearOpMode {

    Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);

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
        }
    }
}
