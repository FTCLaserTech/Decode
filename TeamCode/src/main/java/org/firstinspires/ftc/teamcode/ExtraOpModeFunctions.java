package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static java.lang.Math.abs;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Locale;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;


@Config
public class ExtraOpModeFunctions
{
    public LinearOpMode localLop = null;
    public HardwareMap hm = null;
    public VisionFunctions vision = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum TeamColor{RED,BLUE};
    public enum TrackDepotState{NOTFOUND,TARGETING, TARGETING_ATSPEED, TARGETING_AIMED,ONTARGET};
    public TeamColor teamColor = TeamColor.RED;

    public static final double PI = 3.14159265;

    public DcMotorEx turretMotor;
    public DcMotorEx launcher1;
    public DcMotorEx launcher2;
    public DcMotorEx intake;
    public Servo ballStop;
    public CRServo turretCR;
    public Servo turretS;
    public TouchSensor turretLimitCW;  // Digital channel Object
    public TouchSensor turretLimitCCW;  // Digital channel Object
    public DigitalChannel beamBreak1;
    public DigitalChannel beamBreak2;
    public Lights lights = null;
    public ControlSystem launcherController;
    public static PIDCoefficients launcherVelPidCoefficients =
            new PIDCoefficients(0.0051, 0.0, 0.0);
    public static BasicFeedforwardParameters launcherFeedforwardParameters =
            new BasicFeedforwardParameters(0.00042, 0.0, 0.0);

    public ControlSystem turretController;
    public static PIDCoefficients turretPosPidCoefficients =
            new PIDCoefficients(0.007, 0.0, 0.0);
    public static BasicFeedforwardParameters turretFeedforwardParameters =
            new BasicFeedforwardParameters(0.00042, 0.0, 0.0);

    private AprilTagPoseFtc aprilTagPose;
    private boolean aimGood = false;
    private boolean rangeGood = false;


    double maxLauncherRPM = 4500.0;  //RPM
    double launcherTicksPerRev = 28.0;
    double maxLauncherTPS = launcherTicksPerRev * maxLauncherRPM / 60; // 2800
    //private double shooterTargetVelocity = 0.0;
    double turretGoodAngle = 1.0;
    boolean freezeRange = false;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;
        vision = new VisionFunctions(hardwareMap, localLop);

        turretCR = hardwareMap.get(CRServo.class, "turretCR");
        turretCR.setDirection(DcMotorSimple.Direction.REVERSE);
        turretS = hardwareMap.get(Servo.class, "turretS");
        turretS.setDirection(Servo.Direction.REVERSE);
        turretLimitCW = hardwareMap.get(TouchSensor.class, "turretLimitCW");
        turretLimitCCW = hardwareMap.get(TouchSensor.class, "turretLimitCCW");

        launcher1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        launcher1.setDirection(DcMotorEx.Direction.FORWARD);
        launcher1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher1.setPower(0.0);
        //launcher1.setVelocity(0.0);

        launcher2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        launcher2.setDirection(DcMotorEx.Direction.REVERSE);
        launcher2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setPower(0.0);
        //launcher2.setVelocity(0.0);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocity(0.0);

        lights = new Lights(hardwareMap);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        beamBreak1 = hardwareMap.get(DigitalChannel.class, "beamBreak1");
        beamBreak1.setMode(DigitalChannel.Mode.INPUT);
        beamBreak2 = hardwareMap.get(DigitalChannel.class, "beamBreak2");
        beamBreak2.setMode(DigitalChannel.Mode.INPUT);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        //turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //turretMotor.setVelocity(0.0);

        ballStop = hardwareMap.get(Servo.class, "ballStop");

        launcherController = ControlSystem.builder()
                .velPid(launcherVelPidCoefficients)
                .basicFF(launcherFeedforwardParameters)
                .build();
        launcherController.setGoal(new KineticState(0.0, 0.0));

        turretController = ControlSystem.builder()
                .posPid(turretPosPidCoefficients)
                .basicFF(turretFeedforwardParameters)
                .build();
        turretController.setGoal(new KineticState(0.0, 0.0));
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
        ballStop.setPosition(0.3);
    }

    public void setLauncher(double launcherSpeed)
    {
        launcherController.setGoal(new KineticState(0, launcherSpeed));
        double power = launcherController.calculate(new KineticState(
                launcher1.getCurrentPosition(),
                launcher1.getVelocity()
        ));
        launcher1.setPower(power);
        launcher2.setPower(power);
        localLop.telemetry.addData("Launcher velocity target: ", launcherSpeed);
        localLop.telemetry.addData("Launcher power set: ", power);
        //localLop.telemetry.addData("Launcher1 velocity actual: ", launcher1.getVelocity());
        //localLop.telemetry.addData("Launcher2 velocity actual: ", launcher2.getVelocity());
        //localLop.telemetry.addData("Launcher1 power actual: ", launcher1.getPower());
        //localLop.telemetry.addData("Launcher2 power actual: ", launcher2.getPower());
        //localLop.telemetry.addData("Launcher Speed OK? ", isLauncherSpeedGood(launcherSpeed));

        //dashboardTelemetry.addData("Launcher velocity target", launcherSpeed);
        //dashboardTelemetry.addData("Launcher power set", power);
        //dashboardTelemetry.addData("Launcher1 velocity actual", launcher1.getVelocity());
        //dashboardTelemetry.addData("Launcher2 velocity actual", launcher2.getVelocity());
        //dashboardTelemetry.update();
    }

    public void setTurret(double turretPosition)
    {
        turretController.setGoal(new KineticState(turretPosition));

        double power = turretController.calculate(new KineticState(
                turretMotor.getCurrentPosition(),
                turretMotor.getVelocity()));
        if(power>0.7){
            power = 0.7;
        }
        if(power<-0.7){
            power = -0.7;
        }
        turretMotor.setPower(power);

        localLop.telemetry.addData("NXFTC turretPower", power);

    }

    public double getLauncherSpeed()
    {
        return((launcher1.getVelocity()+ launcher2.getVelocity())/2);
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
    /*
    public void saveAutoStartRotation(Double angle)
    {
        Writer fileWriter;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            fileWriter = new FileWriter(directoryPath+"/START_ROTATION.csv");
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
    public void savePosition(Pose2d pose2d)
    {
        Writer fileWriter;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            fileWriter = new FileWriter(directoryPath+"/SAVED_POSITION.csv");
            String data = String.format(Locale.US, "{%.3f,%.3f,%.3f}", pose2d.position.x,pose2d.position.y,pose2d.heading.toDouble());
            fileWriter.write(data);
            fileWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
    public Pose2d readPosition()
    {
        double x = 0;
        double y = 0;
        double angle = 0;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            BufferedReader br = new BufferedReader(new FileReader(directoryPath+"/SAVED_POSITION.csv"));
            String[] pos = br.readLine().split(",");
            x = Double.parseDouble(pos[1]);
            y = Double.parseDouble(pos[1]);
            angle = Double.parseDouble(pos[1]);
            br.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return(new Pose2d(x,y,angle));
    }
    */

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
    double depotHeading = 0.0;
    //private double turretPower = 0.0;
    SearchingDirection targetSearchingDirection = SearchingDirection.LEFT;
    double targetRange = 0.0;

    public boolean lookForDeopt()
    {
        if(teamColor== ExtraOpModeFunctions.TeamColor.RED)
        {
            aprilTagPose = vision.readRedAprilTag_ll();
        }
        else
        {
            aprilTagPose = vision.readBlueAprilTag_ll();
        }
        if(aprilTagPose == null) // AprilTag not found
        {
            localLop.telemetry.addLine("No April Tag");
            return(false);
        }
        return(true);
    }

    public AprilTagPoseFtc getAprilTagPose()
    {
        return(aprilTagPose);
    }


    public double autoAimTurret()
    {
        double turretPower = 0.0;

        depotHeading = aprilTagPose.bearing - 1;
        localLop.telemetry.addLine("Auto Aim Turret");
        localLop.telemetry.addData("depot heading: ", depotHeading);
        localLop.telemetry.addData("depot elevation: ", aprilTagPose.elevation);

        if (abs(depotHeading) < turretGoodAngle) // depot is in range so turn off turret servo
        {
            turretPower = 0;
            aimGood = true;
        }
        else
        {
            turretPower = angleToSpeed(depotHeading);
            aimGood = false;
        }
        return(turretPower);
    }

    public boolean isTurretAimGood()
    {
        return(aimGood);
    }

    public double limelightLauncherSpeed()
    {
        double shooterTargetVelocity = 0.0;
        targetRange = aprilTagPose.range;
        if(freezeRange == false)
        {
            shooterTargetVelocity = distanceToLauncherSpeed(targetRange);
        }
        return(shooterTargetVelocity);
    }

    public double odometryLauncherSpeed(double goalDistance)
    {
        double shooterTargetVelocity = 0.0;
        if(freezeRange == false)
        {
            shooterTargetVelocity = distanceToLauncherSpeed(goalDistance);
        }
        return(shooterTargetVelocity);
    }

    public double distanceToLauncherSpeed(double distance)
    {
        // range to speed function
        //shooterTargetVelocity = 1049 + (12.8 * targetRange) - (0.0294 * targetRange * targetRange);
        //return( 985 + (13.2 * distance) - (0.024 * distance * distance) );
        return( 1047 + (7.09 * distance) - (0.00119 * distance * distance) );
        //( 1155 + (5 * distance) + (0.00964 * distance * distance) );
    }

    public boolean isLauncherSpeedGood(double shooterTargetVelocity)
    {
        if(abs(getLauncherSpeed() - shooterTargetVelocity) < 50)
        {
            rangeGood = true;
        }
        else
        {
            rangeGood = false;
        }
        return(rangeGood);
    }

    boolean runLauncherBoolean = false;

    public void stopLauncher()
    {
        runLauncherBoolean = false;
    }

    public class SetLauncherAction implements Action
    {
        private double launcherActionSpeed = 0.0;
        private int launcherSpeedCount = 0;

        public SetLauncherAction(double speed)
        {
            launcherActionSpeed = speed;
            runLauncherBoolean = true;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            setLauncher(launcherActionSpeed);
            boolean speedGood = isLauncherSpeedGood(launcherActionSpeed);
            return(runLauncherBoolean);
        }
    }

    public Action setLauncherAction(double speed)
    {
        return(new SetLauncherAction(speed));
    }

    private int intakeFullCount = 0;
    public boolean isIntakeFull()
    {
        localLop.telemetry.addData("beamBreak1: ", beamBreak1.getState());
        localLop.telemetry.addData("beamBreak2: ", beamBreak2.getState());
        localLop.telemetry.addData("intakeFullCount: ", intakeFullCount);
        if((beamBreak1.getState() == false) || (beamBreak2.getState() == false))
        {
            intakeFullCount++;
            if(intakeFullCount > 6)
            {
                lights.flashingOn();
                return(true);
            }
            else
            {
                return(false);
            }
        }
        else
        {
            intakeFullCount = 0;
            lights.flashingOff();
            return(false);
        }
    }

    public class CheckIntakeAction implements Action
    {

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if(isIntakeFull())
            {
                return(false);
            }
            else
            {
                return(true);
            }
        }
    }

    public Action checkIntakeAction()
    {
        return(new CheckIntakeAction());
    }

    public double angleToSpeed(double angle)
    {
        // fitting a parabola through 3 points
        double x1 = turretGoodAngle;  // this is the minimum angle window
        double y1 = 0.13;  // this point sets the minimum speed
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

    public class Lights
    {
        private Servo light1;
        private Servo light2;
        private boolean flashing = false;
        private final long flashPeriod = 250;

        public static final double Light_Green = 0.500;
        public static final double Light_Red = 0.280;
        public static final double Light_Blue = 0.611;
        public static final double Light_Purple = 0.666;
        public static final double Light_Yellow = 0.388;
        public static final double Light_Off = 0.000;
        public double lightColor = Light_Green;

        public Lights(HardwareMap hm)
        {
            this.light1 = hm.get(Servo.class, "light1");
            this.light2 = hm.get(Servo.class, "light2");
            this.setLightColor(Light_Green);
        }
        public void setLightColor(double lightColor)
        {
            this.lightColor = lightColor;
        }

        public void flashingOn()
        {
            if(flashing == false)
            {
                flashing = true;
            }
        }
        public void flashingOff()
        {
            if(flashing == true)
            {
                flashing = false;
            }
        }
        public void lightsUpdate(long timems)
        {
            localLop.telemetry.addData("lights flashing: ", flashing);
            if(flashing)
            {
                //long timeRound = timems - (timems%flashPeriod);
                long timeRound = (long) (timems/flashPeriod);
                localLop.telemetry.addData("timems: ", timems);
                localLop.telemetry.addData("timeRound: ", timeRound);
                localLop.telemetry.addData("timeRound %2: ", (timeRound%2));

                if(false)
                {
                    if ((timeRound % 2) == 0)
                    {
                        light1.setPosition(lightColor);
                        light2.setPosition(lightColor);
                    } else
                    {
                        light1.setPosition(Light_Off);
                        light2.setPosition(Light_Off);
                    }
                }
                else
                {
                    light1.setPosition(Light_Green);
                    light2.setPosition(Light_Green);
                }
            }
            else
            {
                light1.setPosition(Light_Off);
                light2.setPosition(Light_Off);
            }
        }
    }

}

