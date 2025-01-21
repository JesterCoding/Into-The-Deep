package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.HOLD;
import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.IDLE;
import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.INTAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.OUTTAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.RELEASE;
import static org.firstinspires.ftc.teamcode.Subsystems.CONSTANTS.R_IDLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//DO NOT PACKAGE COLOR SENSORS WITH THIS CLASS
//keep all sensor readings localized in the update() function of Robot class
//KEEP BULK READS TO A MINIMUM
public class Intake {
    Servo leftPivot;
    Servo rightPivot;
    Servo release;
    Servo rotation;

    public Intake(HardwareMap hardwareMap) {
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        leftPivot.setDirection(Servo.Direction.REVERSE);
        release = hardwareMap.get(Servo.class, "intake");
        rotation = hardwareMap.get(Servo.class, "rotation");
    }

    public void outtake() {
        rightPivot.setPosition(OUTTAKE);
        leftPivot.setPosition(OUTTAKE);
    }

    public void release() {
        release.setPosition(RELEASE);
    }

    public void hold() {
        release.setPosition(HOLD);
    }

    public void intake() {
        release.setPosition(INTAKE);
    }

    public void idle() {
        rightPivot.setPosition(IDLE);
        leftPivot.setPosition(IDLE);
        release.setPosition(HOLD);
        rotation.setPosition(R_IDLE);
    }

    public void setValue(double position){
        rightPivot.setPosition(position);
        leftPivot.setPosition(position);
    }

}