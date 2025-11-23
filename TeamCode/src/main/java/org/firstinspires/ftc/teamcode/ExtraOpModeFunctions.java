package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static java.lang.Math.abs;

import android.os.Environment;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
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
import java.util.List;
import java.util.concurrent.TimeUnit;


@Config


public class ExtraOpModeFunctions
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum TeamColor{RED,BLUE};
    public TeamColor teamColor = TeamColor.RED;

    public VisionFunctions vision = null;

    public static final double PI = 3.14159265;

    public int elevatorTarget = 0;
    int elevatorResetCounter = 0;

    public Servo s1;
    public Servo s2;
    public Servo s3;
    public CRServo turret;
    public DigitalChannel turretLimit;  // Digital channel Object


    public LinearOpMode localLop = null;
    public HardwareMap hm = null;

    public CRServo intake;


    public boolean firstPressed = true;


    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

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
        vision = new VisionFunctions(hardwareMap, localLop);

        //intake = hardwareMap.get(CRServo.class, "intake");
        //intake.setDirection(CRServo.Direction.FORWARD);
        //intake.setPower(0);

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        turret = hardwareMap.get(CRServo.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLimit = hardwareMap.get(DigitalChannel.class, "turretLimit");
        turretLimit.setMode(DigitalChannel.Mode.INPUT);


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
    public double s4position = 0.5;
    public void s4up()
    {
        if(s4position < 1) {
            s4position += 0.001;
        }
        turret.setPower(s4position);
    }
    public void s4Down()
    {
        if(s4position > 0) {
            s4position -= 0.001;
        }
        turret.setPower(s4position);
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

    public void trackDepot()
    {
        AprilTagPoseFtc aprilTagPose;

        if(teamColor== ExtraOpModeFunctions.TeamColor.RED)
            aprilTagPose = vision.readRedAprilTag_ll();
        else
            aprilTagPose = vision.readBlueAprilTag_ll();

        localLop.telemetry.addData("turret power: ", turretPower);
        localLop.telemetry.addData("target heading: ", targetHeading);
        if( true )
        {
            lastTargetHeading = targetHeading;
            if(aprilTagPose == null) // AprilTag not found
            {
                if(localLop.gamepad2.dpad_left)
                {
                    turretPower = 1.0;
                }
                else if(localLop.gamepad2.dpad_right)
                {
                    turretPower = -1.0;
                }
                else if (localLop.gamepad2.dpad_down)
                {
                    turretPower = 0.0;
                }

                if(turretLimit.getState() == false)
                {
                    turretPower = -turretPower;
                }

            }
            else // AprilTag found
            {
                targetHeading = aprilTagPose.bearing;
                if (abs(targetHeading) < 2.0)
                {
                    turretPower = 0;
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
                    if(limitDirection ==LimitDirection.NA)
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

            }
            turret.setPower(turretPower);
            localLop.telemetry.addData("turret power2: ", turret.getPower());
        }
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

