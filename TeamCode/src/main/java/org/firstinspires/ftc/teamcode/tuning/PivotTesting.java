package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PivotTesting", group = "Custom PID")
public class PivotTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx topPivot, bottomPivot;

    private double TicksToAngle = 249*2/Math.PI;
    private double scalingConst = 0.23;
    private double motorPower;

    public void runOpMode()
    {
        topPivot = hardwareMap.get(DcMotorEx.class, "topRightRotation");
        bottomPivot = hardwareMap.get(DcMotorEx.class, "bottomRightRotation");

        topPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        bottomPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) //249 = pi/2
        {
            double axial =  -gamepad1.left_stick_y;
            int pos = bottomPivot.getCurrentPosition();
            motorPower = scalingConst*Math.cos(TicksToAngle*pos) + axial;
            topPivot.setPower(motorPower);
            bottomPivot.setPower(motorPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pos Data:", pos);
            telemetry.addData("motorPower:", motorPower);
            telemetry.update();
        }
    }


}
