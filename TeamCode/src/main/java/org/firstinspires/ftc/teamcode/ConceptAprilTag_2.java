package org.firstinspires.ftc.teamcode;/* Copyright (c) 2023 FIRST. All rights reserved. */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//import om.self.ezftc.utils.Vector3;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag_2", group = "Concept")
//@Disabled
public class ConceptAprilTag_2 extends LinearOpMode {

    private static final boolean USE_WEBCAM = false;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

//        initAprilTag();
        lkAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */

    private void lkAprilTag() {
        AprilTagMetadata myAprilTagMetadata, myAprilTagMetadata2;
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;
        //AprilTagProcessor myAprilTagProcessor;


// Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
// Get the AprilTagLibrary for the current season.
//        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCenterStageTagLibrary());
//                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())

// Create a new AprilTagMetdata object and assign it to a variable.
        myAprilTagMetadata = new AprilTagMetadata(55, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);
        myAprilTagMetadata2 = new AprilTagMetadata(20, "LAK 36h11 ID20", 5, DistanceUnit.INCH);

//        .addTag(1, "BlueAllianceLeft",
//                2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH,
//                new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
//          .addTag(7, "RedAudienceWallLarge",
//                5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
//                new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))

// Add a tag to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);
        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata2);

// Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

// Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

// Build the AprilTag processor and assign it to a variable.
        //myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        aprilTag = myAprilTagProcessorBuilder.build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }




    private void XinitAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                //Vector3 camOffset = new Vector3(-3.25,4,0);
                Position camOffset = new Position(-6,-2,0);

                // raw camera values (ftcPose in it's native coordinate system) of XY
                Position camRaw = new Position(detection.ftcPose.x, detection.ftcPose.y, 0);
                telemetry.addLine(String.format("camRaw   XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camRaw.X, camRaw.Y, camRaw.R));

                // transform the camera raw position using the yaw to align with field
                Position camTrans = transPos(new Position(0, 0, -detection.ftcPose.yaw), camRaw);
                telemetry.addLine(String.format("camTrans XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camTrans.X, camTrans.Y, camTrans.R));

                // rotate the camera XY 90deg to match the field by switching axes
                Position camRot = new Position(-camTrans.Y, camTrans.X, camTrans.R);
                telemetry.addLine(String.format("camRot   XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camRot.X, camRot.Y, camRot.R));

                // Do everything in one step: Switch ftcPose to field XY and transform by yaw to align with field
                Position camTry2 = transPos(new Position(0,0, -detection.ftcPose.yaw),
                        new Position(-detection.ftcPose.y, detection.ftcPose.x, 0 ));
                telemetry.addLine(String.format("camTry2  XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camTry2.X, camTry2.Y, camTry2.R));

                // Switch ftcPose to field XY relative to tag, add robot offset, and transform by yaw to align with field
                Position camPos = transPos(new Position(0,0, -detection.ftcPose.yaw),
                        new Position(-detection.ftcPose.y + camOffset.X, detection.ftcPose.x + camOffset.Y, 0 ));
                telemetry.addLine(String.format("camPos   XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camPos.X, camPos.Y, camPos.R));

                // Get the tag's field position
                // the library has bad x values!  60.3, but in reality it's 63.5.  So let's add 3.2 inches.
                double adjustment = 0; //3.2;
                float[] fieldPos = detection.metadata.fieldPosition.getData();
                telemetry.addLine(String.format("field XYZ %6.1f %6.1f %6.1f  (inch)", fieldPos[0], fieldPos[1], fieldPos[2]));
                Position tagPos = new Position(detection.metadata.fieldPosition.get(0)+adjustment, detection.metadata.fieldPosition.get(1),0);
                telemetry.addLine(String.format("tagPos   XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", tagPos.X, tagPos.Y, tagPos.R));

                // Calculate the robot position based on camera position and tag position
                Position robotPos = new Position(tagPos.X+camPos.X, tagPos.Y+camPos.Y, camPos.R);
                telemetry.addLine(String.format("robotPos XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", robotPos.X, robotPos.Y, robotPos.R));


//                Vector3 camRaw = new Vector3()

//                //Vector3 camRaw = new Vector3(fieldPos[0] - detection.ftcPose.y, fieldPos[1] + detection.ftcPose.x, -detection.ftcPose.yaw);
//                Vector3 camRaw = new Vector3(fieldPos[0] - detection.ftcPose.y, fieldPos[1] + detection.ftcPose.x, 0);
////                double camX = fieldPos[0] - detection.ftcPose.y;
////                double camY = fieldPos[1] + detection.ftcPose.x;
////                double camR = -detection.ftcPose.yaw;
////                telemetry.addLine(String.format("cam XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camX, camY, camR));
//                telemetry.addLine(String.format("camraw XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camRaw.X, camRaw.Y, camRaw.Z));
//                Vector3 camPos = lkTransformPosition(new Vector3(0,0, -detection.ftcPose.yaw), camRaw);
//                telemetry.addLine(String.format("campos XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camPos.X, camPos.Y, camPos.Z));
//                Vector3 camOffset = new Vector3(-8,0,0);
//                telemetry.addLine(String.format("offset XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", camOffset.X, camOffset.Y, camOffset.Z));
//                Vector3 finalPos = lkTransformPosition(camPos, camOffset);
//                telemetry.addLine(String.format("final  XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", finalPos.X, finalPos.Y, finalPos.Z));


            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        if (currentDetections.size() == 0) {
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
            telemetry.addLine("X");
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

//    void lkUpdateOdoRobotPose() {
//        //pos1 = odoRawPose, pos2 = odoRobotOffset
//        lkOdoRobotPose = lkTransformPosition(lkOdoRawPose, lkOdoRobotOffset);
//    }

    Position transPos(Position pos1, Position pos2) {
        return new Position(
                (pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R)))),
                (pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R)))),
                (pos1.R + pos2.R)
        );
    }

}   // end class
