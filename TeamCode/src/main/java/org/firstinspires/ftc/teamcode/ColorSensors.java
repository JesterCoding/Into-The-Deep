package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp

public class ColorSensors extends LinearOpMode {

    private ColorSensor color;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            int red = color.red();
            int blue = color.blue();
            int yellow = color.green();

            telemetry.addData("Red: ", red);
            telemetry.addData("Blue: ", blue);
            telemetry.addData("Yellow: ", yellow);

            if (red > blue && red > yellow)
            {
                telemetry.addData("Color Detected: ", "red");
            }
            else if (blue > red && blue > yellow) {

                telemetry.addData("Color Detected: ", "blue");
            }
            else if (yellow > red && yellow > blue)
            {
                telemetry.addData("Color Detected: ", "yellow");
            }
            else
            {
                telemetry.addData("Color Detected: ", "unknown");
            }

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}