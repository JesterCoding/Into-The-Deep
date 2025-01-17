package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;


public class AprilTags {
    AprilTagProcessor processor;

    AprilTagLibrary.Builder libraryBuilder;
    AprilTagProcessor.Builder processorBuilder;
    AprilTagLibrary library;

    private VisionPortal visionPortal;

    List<AprilTagDetection> currentDetections;

    public void initAprilTag(HardwareMap hardwareMap, String device) {
        libraryBuilder = new AprilTagLibrary.Builder();
        libraryBuilder.addTags(CSTagLibrary.futureCenterStageTagLibrary());
        library = libraryBuilder.build();

        processorBuilder = new AprilTagProcessor.Builder();
        processorBuilder.setTagLibrary(library);
        //.setDrawAxes(false)
        processorBuilder.setDrawCubeProjection(true);
        processorBuilder.setDrawTagOutline(true);
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        processorBuilder.setLensIntrinsics(463.108, 463.108, 316.448, 252.025); // calibration for no name camera
        processor = processorBuilder.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, device))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void updateTag() {
        currentDetections = processor.getDetections();
    }

    @SuppressLint("DefaultLocale")
    public void telemetryAprilTag(Telemetry telemetry) {
        if (currentDetections != null) {

            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

        }
    }// end method telemetryAprilTag()

    public ArrayList<Integer> getDetectedIDs() {
        ArrayList<Integer> ids = new ArrayList<>();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                ids.add(detection.id);
            }
        }
        return ids;
    }
}