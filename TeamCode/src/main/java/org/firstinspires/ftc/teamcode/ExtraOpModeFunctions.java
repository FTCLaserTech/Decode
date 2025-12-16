package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static java.lang.Math.abs;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;


@Config


public class ExtraOpModeFunctions
{
    public LinearOpMode localLop = null;
    public HardwareMap hm = null;
    public VisionFunctions vision = null;

    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum TeamColor{RED,BLUE};
    public enum TrackDepotState{NOTFOUND,TARGETING, TARGETING_ATSPEED, TARGETING_AIMED,ONTARGET};
    public TeamColor teamColor = TeamColor.RED;

    public static final double PI = 3.14159265;

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotorEx intake;
    public Servo ballStop;
    public CRServo turret;
    public TouchSensor turretLimitCW;  // Digital channel Object
    public TouchSensor turretLimitCCW;  // Digital channel Object
    public Servo light1;
    public Servo light2;
    private ControlSystem controller;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.001, 0.0, 0.0);

    public static double Light_Green = 0.500;
    public static double Light_Red = 0.280;
    public static double Light_Blue = 0.611;
    public static double Light_Purple = 0.666;
    public static double Light_Yellow = 0.388;
    double lightColor = Light_Red;


    double maxShooterRPM = 6000.0;  //RPM
    double shooterTicksPerRev = 28.0;
    double maxShooterTPS = shooterTicksPerRev * maxShooterRPM / 60; // 2800
    double shooterVelocity = 0.0;
    double turretGoodAngle = 1.0;
    boolean freezeRange = false;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;
        vision = new VisionFunctions(hardwareMap, localLop);

        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLimitCW = hardwareMap.get(TouchSensor.class, "turretLimitCW");
        turretLimitCCW = hardwareMap.get(TouchSensor.class, "turretLimitCCW");

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

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocity(0.0);

        ballStop = hardwareMap.get(Servo.class, "ballStop");

        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light1.setPosition(Light_Green);
        light2.setPosition(Light_Green);

        controller = ControlSystem.builder()
                .velPid(coefficients)
                .build();

        controller.setGoal(new KineticState(0.0, 0.0));
    }

    public void intakeForward()
    {
        intake.setPower(1.0);
    }

    public void intakeOff()
    {
        intake.setPower(0.0);
    }

    public void intakeReverse()
    {
        intake.setPower(-1.0);
    }

    public void ballStopOn()
    {
        ballStop.setPosition(1.0);
    }

    public void ballStopOff()
    {
        ballStop.setPosition(0.5);
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
            fileWriter = new FileWriter(directoryPath+"/.csv");
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
            BufferedReader br = new BufferedReader(new FileReader(directoryPath+"/START_ROTATION.csv"));
            angle = Double.parseDouble(br.readLine());
            br.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return(angle);
    }
    public void saveTeamColor(TeamColor teamColor)
    {
        Writer fileWriter;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            fileWriter = new FileWriter(directoryPath+"/TEAM_COLOR.csv");
            fileWriter.write(teamColor.toString());
            fileWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
    public TeamColor readTeamColor()
    {
        TeamColor teamColor = TeamColor.RED;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            BufferedReader br = new BufferedReader(new FileReader(directoryPath+"/TEAM_COLOR.csv"));
            teamColor = TeamColor.valueOf(br.readLine());
            br.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return(teamColor);
    }

    public enum SearchingDirection {LEFT,RIGHT};
    double targetHeading = 0.0;
    double turretPower = 0.0;
    SearchingDirection targetSearchingDirection = SearchingDirection.LEFT;
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

        if(aprilTagPose == null) // AprilTag not found
        {
            localLop.telemetry.addLine("No April Tag");
            if(turretLimitCW.isPressed())
            {
                turretPower = -abs(turretPower);
            }

            if(turretLimitCCW.isPressed())
            {
                turretPower = abs(turretPower);
            }

            lightColor = Light_Red;
        }
        else // AprilTag found
        {
            localLop.telemetry.addLine("April Tag Found");

            targetHeading = aprilTagPose.bearing;
            localLop.telemetry.addData("target heading: ", targetHeading);
            localLop.telemetry.addData("target elevation: ", aprilTagPose.elevation);

            if (abs(targetHeading) < turretGoodAngle)
            {
                turretPower = 0;
                aimGood = true;
            }
            else
            {
                //turretPower = (1.0-0.05)/(45.0-2.0)*(targetHeading-2.0)+0.05;
                turretPower = angleToSpeed(targetHeading);
                //if(turretPower > 1.0)
                //    turretPower = 1.0;
            }

            if((turretLimitCW.isPressed())&&(turretPower>0))
            {
                turretPower = 0.0;
                //turretPower = -turretPower;
            }
            else if((turretLimitCCW.isPressed())&&(turretPower<0))
            {
                turretPower = 0.0;
                //turretPower = -turretPower;
            }

            // range
            targetRange = aprilTagPose.range;
            localLop.telemetry.addData("targetRange: ", targetRange);
            // insert range to speed function
            if(freezeRange == false)
            {
                shooterVelocity = 1049 + (12.8 * targetRange) - (0.0294 * targetRange * targetRange);
                controller.setGoal(new KineticState(0, shooterVelocity));

            }
            setShooter(controller.calculate(new KineticState(
                    shooter1.getCurrentPosition(),
                    shooter1.getVelocity()
            )));
            //setShooter(shooterVelocity);
            localLop.telemetry.addData("Shooter velocity set: ", shooterVelocity);
            localLop.telemetry.addData("Shooter1 velocity actual: ", shooter1.getVelocity());
            localLop.telemetry.addData("Shooter2 velocity actual: ", shooter2.getVelocity());            if(abs(getShooter()-shooterVelocity) < 50)
            {
                rangeGood = true;
            }

            if (aimGood && rangeGood)
            {
                trackDepotState = TrackDepotState.ONTARGET;
                lightColor = Light_Green;
            }
            else if (aimGood)
            {
                trackDepotState = TrackDepotState.TARGETING_AIMED;
                lightColor = Light_Purple;
            }
            else if (rangeGood)
            {
                trackDepotState = TrackDepotState.TARGETING_ATSPEED;
                lightColor = Light_Blue;
            }
            else
            {
                trackDepotState = TrackDepotState.TARGETING;
                lightColor = Light_Blue;
            }
        }
        light1.setPosition(lightColor);
        light2.setPosition(lightColor);
        turret.setPower(turretPower);
        localLop.telemetry.addData("turret power2: ", turret.getPower());

        return (trackDepotState);
    }

    public class TrackDepotAction implements Action
    {
        public boolean isComplete = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            trackDepot();
            return(isComplete);
        }

        public void setComplete()
        {
            isComplete = true;
        }
    }

    public double angleToSpeed(double angle)
    {
        // fitting a parabola through 3 points
        double x1 = turretGoodAngle;  // this is the minimum angle window
        double y1 = 0.13;  // this point sets the minimum speed close to
        double x2 = 12.0;  // this angle has the maximum speed
        double y2 = 0.5;  // this is the maximum speed - a servo max is 1
        double x3 = -x1;
        double y3 = y1;

        double y = y1*(angle-x2)*(angle-x3)/((x1-x2)*(x1-x3))
                +y2*(angle-x1)*(angle-x3)/((x2-x1)*(x2-x3))
                +y3*(angle-x1)*(angle-x2)/((x3-x1)*(x3-x2));

        if(y>y2)
        {
            y = y2;
        }
        if(angle>0)
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

