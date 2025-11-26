package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static java.lang.Math.abs;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.sql.Time;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Config


public class ExtraOpModeFunctions
{
    public LinearOpMode localLop = null;
    public HardwareMap hm = null;
    public VisionFunctions vision = null;

    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum TeamColor{RED,BLUE};
    public enum TrackDepotState{NOTFOUND,TARGETING,ONTARGET};
    public TeamColor teamColor = TeamColor.RED;

    public static final double PI = 3.14159265;

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotorEx intake1;
    public DcMotorEx intake2;
    public CRServo turret;
    public DigitalChannel turretLimit;  // Digital channel Object
    public Servo light;

    public static double Light_Green = 0.500;
    public static double Light_Red = 0.280;
    public static double Light_Blue = 0.611;
    public static double Light_Purple = 0.666;


    double maxShooterRPM = 6000.0;  //RPM
    double shooterTicksPerRev = 28.0;
    double maxShooterTPS = shooterTicksPerRev * maxShooterRPM / 60; // 2800
    double shooterVelocity = 0.0;


    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;
        vision = new VisionFunctions(hardwareMap, localLop);

        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLimit = hardwareMap.get(DigitalChannel.class, "turretLimit");
        turretLimit.setMode(DigitalChannel.Mode.INPUT);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(0.0);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setVelocity(0.0);

        intake1 = hardwareMap.get(DcMotorEx.class, "intakeIn");
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setVelocity(0.0);

        intake2 = hardwareMap.get(DcMotorEx.class, "intakeOut");
        intake2.setDirection(DcMotorEx.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setVelocity(0.0);

        light = hardwareMap.get(Servo.class, "light");

    }


    public void intake1Forward()
    {
        intake1.setPower(1);
    }
    public void intake1Reverse()
    {
        intake1.setPower(-1);
    }
    public void intake1Off()
    {
        intake1.setPower(0);
    }

    public void intake2Forward() {
        intake2.setPower(1);
    }
    public void intake2Reverse() {
        intake2.setPower(-1);
    }
    public void intake2Off()
    {
        intake2.setPower(0);
    }

    public void intakeForward()
    {
        intake1Forward();
        intake2Forward();
    }

    public void intakeOff()
    {
        intake1Off();
        intake2Off();
    }

    public void setShooter(double shooterSpeed)
    {
        shooter1.setVelocity(shooterSpeed);
        shooter2.setVelocity(shooterSpeed);
    }

    public double getShooter()
    {
        return((shooter1.getVelocity()+shooter2.getVelocity())/2);
    }

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

    public enum SearchingDirection {LEFT,RIGHT};
    public enum LimitDirection {NA,CW,CCW};
    double targetHeading = 0.0;
    double lastTargetHeading = 0.0;
    double turretPower = 0.5;
    SearchingDirection targetSearchingDirection = SearchingDirection.LEFT;
    LimitDirection limitDirection = LimitDirection.NA;
    double targetRange = 0.0;

    public TrackDepotState trackDepot()
    {
        AprilTagPoseFtc aprilTagPose;
        boolean aimGood = false;
        boolean rangeGood = false;
        TrackDepotState trackDepotState = TrackDepotState.NOTFOUND;

        if(teamColor== ExtraOpModeFunctions.TeamColor.RED)
            aprilTagPose = vision.readRedAprilTag_ll();
        else
            aprilTagPose = vision.readBlueAprilTag_ll();

        localLop.telemetry.addData("turret power: ", turretPower);
        localLop.telemetry.addData("target heading: ", targetHeading);

        lastTargetHeading = targetHeading;
        if(aprilTagPose == null) // AprilTag not found
        {
            if(turretLimit.getState() == false)
            {
                turretPower = -turretPower;
            }
        }
        else // AprilTag found
        {
            if (abs(targetHeading) < 2.0)
            {
                turretPower = 0;
                aimGood = true;
            }
            else
            {
                //turretPower = (1.0-0.05)/(45.0-2.0)*(targetHeading-2.0)+0.05;
                turretPower = angleToSpeed(targetHeading);
                if(turretPower > 1.0)
                    turretPower = 1.0;
            }

            if(turretLimit.getState() == false)
            {
                if(limitDirection == LimitDirection.NA)
                {
                    if(targetHeading > 0)
                    {
                        limitDirection = LimitDirection.CW;
                    }
                    else
                    {
                        limitDirection = LimitDirection.CCW;
                    }
                }
                if((limitDirection == LimitDirection.CW)&&(targetHeading>0))
                {
                    turretPower = 0.0;
                }
                else if((limitDirection ==LimitDirection.CCW)&&(targetHeading<0))
                {
                    turretPower = 0.0;
                }
            }
            else
            {
                limitDirection =LimitDirection.NA;
            }

            // range
            targetRange = aprilTagPose.range;
            // insert range to speed function
            double shooterSpeed = 0.0;
            setShooter(shooterSpeed);
            if(abs(getShooter()-shooterSpeed) < 50)
            {
                rangeGood = true;
            }

            if (aimGood && rangeGood)
            {
                trackDepotState = TrackDepotState.ONTARGET;
            }
            else
            {
                trackDepotState = TrackDepotState.TARGETING;
            }
        }
        turret.setPower(turretPower);
        localLop.telemetry.addData("turret power2: ", turret.getPower());

        return (trackDepotState);
    }

    public double angleToSpeed(double angle)
    {
        double x1 = 2.0;
        double y1 = 0.05;
        double x2 = 24.0;
        double y2 = 1.0;
        double x3 = -x1;
        double y3 = y1;

        double y = y1*(angle-x2)*(angle-x3)/((x1-x2)*(x1-x3))
                +y2*(angle-x1)*(angle-x3)/((x2-x1)*(x2-x3))
                +y3*(angle-x1)*(angle-x2)/((x3-x1)*(x3-x2));

        if(angle<0)
            y = -y;
        return(y);
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField cameraPosition = new Datalogger.GenericField("Camera Position");
        public Datalogger.GenericField targetHeading = new Datalogger.GenericField("Target Heading");
        public Datalogger.GenericField imuHeading         = new Datalogger.GenericField("IMU Heading");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            loopCounter,
                            cameraPosition,
                            targetHeading,
                            imuHeading
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }

}

