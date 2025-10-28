package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.List;


@Config


public class ExtraOpModeFunctions
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};

    public enum ElevatorPosition {INIT, DOWN, HIGH_CHAMBER, HIGH_BASKET, LOW_BASKET, LOW_CHAMBER}
    public ElevatorPosition elevatorPosition = ElevatorPosition.INIT;

    public static final double PI = 3.14159265;

    public int elevatorTarget = 0;
    int elevatorResetCounter = 0;

    public Servo s1;
    public Servo s2;
    public Servo s3;
    public LinearOpMode localLop = null;

    public CRServo intake;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;
    public enum ObeliskPattern {GPP,PGP,PPG};

    public boolean firstPressed = true;

    public NormalizedColorSensor colorSensor1;
    public NormalizedColorSensor colorSensor2;
    public NormalizedColorSensor colorSensor3;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public HardwareMap hm = null;
    
    public enum BasketDelivery {IDLE, ARMUP, WAIT1, INTAKEOUT, WAIT2, INTAKEOFF}
    BasketDelivery basketDeliveryState = BasketDelivery.IDLE;

    public enum Collect {IDLE, DUMP, WAIT, COLLECT}
    Collect dumpState = Collect.IDLE;

    public enum Elevator {IDLE, HALF, WAIT, DOWN}
    Elevator elevatorState = Elevator.IDLE;

    public enum SpecimenPickupStates {IDLE, PICKUP, WAIT, DROPTAIL}
    SpecimenPickupStates specimenPickupState = SpecimenPickupStates.IDLE;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;

        //intake = hardwareMap.get(CRServo.class, "intake");
        //intake.setDirection(CRServo.Direction.FORWARD);
        //intake.setPower(0);

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs0");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        float gain = (float)1.0;
        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);
        colorSensor3.setGain(gain);

        initAprilTag();

    }



    public void s1up()
    {
        s1.setPosition(0);
    }

    public void s1down()
    {
        s1.setPosition(1);
    }
    public void s2up()
    {
        s2.setPosition(0);
    }

    public void s2down()
    {
        s2.setPosition(1);
    }
    public void s3up()
    {
        s3.setPosition(0);
    }

    public void s3down()
    {
        s3.setPosition(1);
    }


    public void intakeIn() {
        intake.setPower(1);
    }
    public void intakeOut() {
        intake.setPower(-1);
    }
    public void intakeOff()
    {
        intake.setPower(0);
    }

    public enum ArmPosition {STOP, EXTEND, RETRACT, HORIZONTAL, VERTICAL, HANG1, HANG2,HOME}
    ArmPosition armPosition = ArmPosition.STOP;

    /*
    public void setElevatorPosition(int target)
    {
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);
        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }
    */

    public boolean armMoving = false;
    public int step = 0;

    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

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
        //aprilTag.setDecimation(3);

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

    }   // end method initAprilTag()


    /**
     * Read the Obelisk
     */
    public ObeliskPattern readObelisk() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        localLop.telemetry.addData("# AprilTags Detected", currentDetections.size());
        ObeliskPattern obelisk = ObeliskPattern.GPP;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            /*if (detection.metadata != null) {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                localLop.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                localLop.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                localLop.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                localLop.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

             */
            if(detection.id==21) {
                obelisk = ObeliskPattern.GPP;
                localLop.telemetry.addLine("GPP");
            }
            else if(detection.id==22) {
                obelisk = ObeliskPattern.PGP;
                localLop.telemetry.addLine("PGP");
            }
            else if(detection.id==23) {
                obelisk = ObeliskPattern.PPG;
                localLop.telemetry.addLine("PPG");
            }

        }   // end for() loop

        // Add "key" information to telemetry
        //localLop.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //localLop.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //localLop.telemetry.addLine("RBE = Range, Bearing & Elevation");

        return obelisk;

    }   // end method telemetryAprilTag()

    /**
     * Read the Obelisk
     */
    public double readRedBearing() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        localLop.telemetry.addData("# AprilTags Detected", currentDetections.size());
        double bearing = -1.0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            /*if (detection.metadata != null) {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                localLop.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                localLop.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                localLop.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                localLop.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                localLop.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
             */

            if(detection.id==24) {
                bearing = detection.ftcPose.bearing;
                localLop.telemetry.addData("Red Bearing: ", detection.ftcPose.bearing);
            }

        }   // end for() loop

        // Add "key" information to telemetry
        //localLop.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //localLop.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //localLop.telemetry.addLine("RBE = Range, Bearing & Elevation");

        return bearing;

    }   // end method telemetryAprilTag()

    private static final String BASE_FOLDER_NAME = "Team14631";
    private static final String TEAM_LOG = "Team14631";
    public void saveAutoStartRotation(Double angle)
    {
        Writer fileWriter;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            fileWriter = new FileWriter(directoryPath+"/"+TEAM_LOG+".csv");
            fileWriter.write(angle.toString());
            fileWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
    public double readAutoStartRotation()
    {
        double angle = 0;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            BufferedReader br = new BufferedReader(new FileReader(directoryPath+"/"+TEAM_LOG+".csv"));
            angle = Double.parseDouble(br.readLine());
            br.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return(angle);
    }
}

