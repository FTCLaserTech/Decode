package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;


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



    public boolean firstPressed = true;

    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

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
    /*
    public void s2()
    {
        s2.setPosition(-1);
    }

    public void s2()
    {
        s2.setPosition(1);
    }
    public void s3()
    {
        s3.setPosition(-1);
    }

    public void s3()
    {
        s3.setPosition(1);
    }
*/
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
}

