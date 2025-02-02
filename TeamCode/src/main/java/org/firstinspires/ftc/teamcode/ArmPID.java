package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "ArmPowerTesting", group = "Custom PID")
@Disabled
public class ArmPID extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor topRightRotation = null;
    private DcMotor bottomRightRotation = null;

    private int posBottom = 180;
    //private int posTop = 235;

    public void runOpMode()
    {
        topRightRotation = hardwareMap.get(DcMotor.class, "topRightRotation");
        bottomRightRotation = hardwareMap.get(DcMotor.class, "bottomRightRotation");

        //topRightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topRightRotation.setDirection(DcMotor.Direction.FORWARD);
        bottomRightRotation.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset(); //9651 //9497

        while (opModeIsActive()) {

            bottomRightRotation.setTargetPosition(posBottom);
            bottomRightRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomRightRotation.setPower(1.0);

            //topRightRotation.setPower(bottomRightRotation.getPower());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Top Encoder Data:", topRightRotation.getCurrentPosition());
            telemetry.addData("Bottom Encoder Data:", bottomRightRotation.getCurrentPosition());
            telemetry.update();
        }
    }
}
