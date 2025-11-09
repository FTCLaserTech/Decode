package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Config


public class VisionFunctions {
    public static final double PI = 3.14159265;

    public LinearOpMode localLop = null;

    public CRServo intake;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;

    public enum ObeliskPattern {GPP, PGP, PPG}

    private Limelight3A limelight;

    private IMU imu;

    private long loopCounter = 0;

    ;

    public boolean firstPressed = true;

    public NormalizedColorSensor colorSensor1;
    public NormalizedColorSensor colorSensor2;
    public NormalizedColorSensor colorSensor3;

    public HardwareMap hm = null;

    public VisionFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        hm = hardwareMap;
        localLop = linearOpMode;

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs0");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        float gain = (float) 1.0;
        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);
        colorSensor3.setGain(gain);

        initCameraAprilTag();
        initLimelight();

    }

    public void initLimelight() {
        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();;
        /*
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        */
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initCameraAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
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
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hm.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 800));

        //Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
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

        //setManualExposure(10,50);
    }   // end method initAprilTag()

    /*
         Manually set the camera gain and exposure.
         Can only be called AFTER calling initAprilTag();
         Returns true if controls are set.
      */
    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            localLop.telemetry.addData("Camera", "Waiting");
            localLop.telemetry.update();
            while (!localLop.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                localLop.sleep(20);
            }
            localLop.telemetry.addData("Camera", "Ready");
            localLop.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!localLop.isStopRequested()) {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                localLop.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            localLop.sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            localLop.sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /**
     * Read the Obelisk with the Limelight
     */
    public ObeliskPattern readObeliskLimelight() {
        ObeliskPattern obelisk = ObeliskPattern.GPP;
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == 21) {
                obelisk = ObeliskPattern.GPP;
                localLop.telemetry.addLine("LL GPP");
            } else if (id == 22) {
                obelisk = ObeliskPattern.PGP;
                localLop.telemetry.addLine("LL PGP");
            } else if (id == 23) {
                obelisk = ObeliskPattern.PPG;
                localLop.telemetry.addLine("LL PPG");
            }

        }
        return obelisk;

    }

    /**
     * Read the Obelisk with the camera
     */
    public ObeliskPattern readObeliskCamera() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        localLop.telemetry.addData("# AprilTags Detected", currentDetections.size());
        ObeliskPattern obelisk = ObeliskPattern.GPP;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            /*
            if (detection.metadata != null) {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                localLop.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                localLop.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                localLop.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                localLop.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            */

            if (detection.id == 21) {
                obelisk = ObeliskPattern.GPP;
                localLop.telemetry.addLine("Cam GPP");
            } else if (detection.id == 22) {
                obelisk = ObeliskPattern.PGP;
                localLop.telemetry.addLine("Cam PGP");
            } else if (detection.id == 23) {
                obelisk = ObeliskPattern.PPG;
                localLop.telemetry.addLine("Cam PPG");
            }

        }   // end for() loop

        // Add "key" information to telemetry
        //localLop.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //localLop.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //localLop.telemetry.addLine("RBE = Range, Bearing & Elevation");

        return obelisk;

    }   // end method readObeliskCamera()

    /**
     * Read the Red AprilTag with the camera
     */
    public AprilTagPoseFtc readRedAprilTag_cam() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        localLop.telemetry.addData("# AprilTags Detected", currentDetections.size());
        AprilTagPoseFtc pose = null;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24) {
                pose = detection.ftcPose;
                localLop.telemetry.addLine("Red Camera ");
                //printpose(pose);
            }
        }   // end for() loop
        return pose;

    }   // end method readRedAprilTag_cam()/**

    /**
     * Read the Blue AprilTag with the camera
     */
    public AprilTagPoseFtc readBlueAprilTag_cam() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        localLop.telemetry.addData("# AprilTags Detected", currentDetections.size());
        AprilTagPoseFtc pose = null;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20) {
                pose = detection.ftcPose;
                localLop.telemetry.addLine("Blue Camera ");
                printpose(pose);
            }

        }   // end for() loop
        return pose;

    }   // end method readBlueAprilTag_cam()

    /**
     * Read the Red AprilTag with the camera
     */
    public AprilTagPoseFtc readRedAprilTag_ll()
    {
        AprilTagPoseFtc pose = null;
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == 24) {
                Pose3D pose3d = fiducial.getTargetPoseCameraSpace();
                //Pose3D pose3d = fiducial.getTargetPoseRobotSpace();
                pose = pose3D_to_AprilTagPoseFtc(pose3d);

                localLop.telemetry.addLine("Red Limelight ");
                //printpose(pose);
            }
        }
        return pose;
    }   // end method readRedAprilTag_cam()/**

    /**
     * Read the Blue AprilTag with the camera
     */
    public AprilTagPoseFtc readBlueAprilTag_ll()
    {
        AprilTagPoseFtc pose = null;
        LLResult llResult = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == 20) {
                Pose3D pose3d = fiducial.getTargetPoseRobotSpace();
                pose = pose3D_to_AprilTagPoseFtc(pose3d);

                localLop.telemetry.addLine("Blue Limelight ");
                printpose(pose);
            }
        }
        return pose;
    }   // end method readBlueAprilTag_cam()

    public AprilTagPoseFtc pose3D_to_AprilTagPoseFtc(Pose3D pose3d)
    {
        AprilTagPoseFtc pose = new AprilTagPoseFtc(pose3d.getPosition().x*1000/25.4,
                pose3d.getPosition().z*1000/25.4,
                -pose3d.getPosition().y*1000/25.4,
                -pose3d.getOrientation().getPitch(),
                pose3d.getOrientation().getRoll(),
                pose3d.getOrientation().getYaw(),
                Math.hypot(pose3d.getPosition().x, pose3d.getPosition().y),
                Math.atan2(-pose3d.getPosition().x, pose3d.getPosition().y),
                Math.atan2(pose3d.getPosition().z, pose3d.getPosition().y));

        return(pose);
    }

    public void printpose(AprilTagPoseFtc pose)
    {

        localLop.telemetry.addData("x ", pose.x);
        localLop.telemetry.addData("y ", pose.y);
        localLop.telemetry.addData("z: ", pose.z);
        localLop.telemetry.addData("yaw: ", pose.yaw);
        localLop.telemetry.addData("pitch:", pose.pitch);
        localLop.telemetry.addData("roll: ", pose.roll);
        localLop.telemetry.addData("Bearing: ", pose.bearing);
        localLop.telemetry.addData("Range: ", pose.range);
        localLop.telemetry.addData("Elevation: ", pose.elevation);

    }

}

