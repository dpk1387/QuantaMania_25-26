package org.firstinspires.ftc.teamcode;

import android.util.Size;

import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTag extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double tagSize = 6.4375;
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Tag20", tagSize, DistanceUnit.INCH)
                .addTag(21, "Tag21", tagSize, DistanceUnit.INCH)
                .addTag(22, "Tag22", tagSize, DistanceUnit.INCH)
                .addTag(23, "Tag23", tagSize, DistanceUnit.INCH)
                .addTag(24, "Tag24", tagSize, DistanceUnit.INCH)
                .build();


        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor) //add created tag processor
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No tags detected");
            }

            else {
                //AprilTagDetection tag = tagProcessor.getDetections().get(0);
                for (AprilTagDetection tag : detections) {
                    //tag is first detected tag
                    telemetry.addData("Tag ID", tag.id);

                    if (tag.ftcPose == null) {
                        telemetry.addLine("Pose not available for this tag");
                    }
                    else {
                        double x = tag.ftcPose.x;
                        double y = tag.ftcPose.y;
                        double z = tag.ftcPose.z;
                        double roll = tag.ftcPose.roll;
                        double pitch = tag.ftcPose.pitch;
                        double yaw = tag.ftcPose.yaw;
                        double bearing = tag.ftcPose.bearing;
                        double elevation = tag.ftcPose.elevation;
                        double range = tag.ftcPose.range;

                        telemetry.addData("XYZ", String.format(Locale.US, "%.2f %.2f %.2f", x, y, z));
                        telemetry.addData("RPY", String.format(Locale.US, "%.2f %.2f %.2f", roll, pitch, yaw));
                        telemetry.addData("RBE", String.format(Locale.US, "%.2f %.2f %.2f", range, bearing, elevation));
                        telemetry.addLine("");
                    }
                }
            }

            telemetry.update();
            sleep(20);
        }
    }
}
