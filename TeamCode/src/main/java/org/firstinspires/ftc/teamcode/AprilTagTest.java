package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group="testing")
public class AprilTagTest extends LinearOpMode {

    AprilTags aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag = new AprilTags();
        aprilTag.initAprilTag(hardwareMap, "Webcam 1");
        waitForStart();

        while(opModeIsActive()) {
            aprilTag.updateTag();
            aprilTag.telemetryAprilTag(telemetry);
            telemetry.update();
        }
    }
}