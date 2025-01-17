package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class CSTagLibrary {
    public static AprilTagLibrary futureCenterStageTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(7,"tag7",
                        0.127, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(8,"tag8",
                        0.0508, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(9,"tag9",
                        0.0508, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(10,"tag10",
                        0.127, new VectorF(0,0,0), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .build();

    }
}