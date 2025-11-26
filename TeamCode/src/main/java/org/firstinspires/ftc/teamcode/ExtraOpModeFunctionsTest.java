package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;


@Config


public class ExtraOpModeFunctionsTest
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public static final double PI = 3.14159265;

    public LinearOpMode localLop = null;
    public HardwareMap hm = null;

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public TouchSensor beamBreak;

    public boolean firstPressed = true;

    public ExtraOpModeFunctionsTest(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;

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

        beamBreak = hardwareMap.get(TouchSensor.class, "beamBreak");
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
}

