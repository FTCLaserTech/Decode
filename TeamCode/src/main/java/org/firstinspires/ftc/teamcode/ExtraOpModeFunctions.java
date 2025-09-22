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

    public Servo elbow;
    public LinearOpMode localLop = null;

    public DcMotorEx elevator;
    public CRServo intake;
    public DcMotorEx arm;
    public TouchSensor elevatorLimit;
    public Servo tail;
    public Servo dumper;
    public DcMotorEx lift;



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

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(0);

        tail = hardwareMap.get(Servo.class, "tail");

        dumper = hardwareMap.get(Servo.class, "dumper");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.1);


        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10.0, 0.05, 0.0, 0.0);
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        elevator.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setCurrentAlert(6, CurrentUnit.AMPS);
        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
    }

    ElapsedTime basketDeliveryStateTimer = new ElapsedTime(MILLISECONDS);
    public void basketDeliveryStateMachine() {
        switch (basketDeliveryState) {
            case IDLE:
                //do nothing
                break;
            case ARMUP:
                armRetract();
                intakeIn();
                tailDown();
                basketDeliveryStateTimer.reset();
                basketDeliveryState = BasketDelivery.WAIT1;
                break;
            case WAIT1:
                if (basketDeliveryStateTimer.milliseconds() > 700) {
                    basketDeliveryState = BasketDelivery.INTAKEOUT;
                }
                break;
            case INTAKEOUT:
                intakeOut();
                basketDeliveryStateTimer.reset();
                basketDeliveryState = BasketDelivery.WAIT2;
                break;
            case WAIT2:
                if (basketDeliveryStateTimer.milliseconds() > 400) {
                    basketDeliveryState = BasketDelivery.INTAKEOFF;
                }
                break;
            case INTAKEOFF:
                intakeOff();
                elevatorHighBasket();
                basketDeliveryState = BasketDelivery.IDLE;
                break;
        }
    }

    ElapsedTime dumpStateTimer = new ElapsedTime(MILLISECONDS);
    public void collectStateMachine() {
        switch (dumpState) {
            case IDLE:
                //do nothing
                break;
            case DUMP:
                dumper.setPosition(1.0);
                dumpStateTimer.reset();
                dumpState = Collect.WAIT;
                break;
            case WAIT:
                if (dumpStateTimer.milliseconds() > 1200) {
                    dumpState = Collect.COLLECT;
                }
                break;
            case COLLECT:
                dumper.setPosition(-1.0);
                dumpState = Collect.IDLE;
                break;
        }
    }
    /*
    ElapsedTime elevatorStateTimer = new ElapsedTime(MILLISECONDS);
    public void elevatorStateMachine() {
        switch (elevatorState) {
            case IDLE:
                //do nothing
                break;
            case HALF:
                elevatorHalf();
                elevatorStateTimer.reset();
                elevatorState = Collect.WAIT;
                break;
            case WAIT:
                if (elevatorStateTimer.milliseconds() > 1000) {
                    dumpState = Collect.COLLECT;
                }
                break;
            case DOWN:
                dumper.setPosition(-1.0);
                dumpState = Collect.IDLE;
                break;
        }
    }
    */
    ElapsedTime specimenPickupStateTimer = new ElapsedTime(MILLISECONDS);
    public void specimenPickupStateMachine() {
        switch (specimenPickupState) {
            case IDLE:
                //do nothing
                break;
            case PICKUP:
                armVertical();
                elevatorHighChamber();
                specimenPickupStateTimer.reset();
                specimenPickupState = SpecimenPickupStates.WAIT;
                break;
            case WAIT:
                if (specimenPickupStateTimer.milliseconds() > 1000)
                {
                    specimenPickupState = SpecimenPickupStates.DROPTAIL;
                }
                break;
            case DROPTAIL:
                tailDown();
                specimenPickupState = SpecimenPickupStates.IDLE;
                break;
        }
    }

    public void initElevator()
    {
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(0.2);
        localLop.sleep(600);
        elevator.setPower(0);
        localLop.telemetry.addData("Limit ", elevatorLimit.getValue());
        localLop.sleep(100);
        elevator.setPower(-0.1);

        while(elevatorLimit.isPressed())
        {
            ;
        }

        elevator.setPower(0);
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        localLop.sleep(250);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        localLop.telemetry.addLine("Elevator Initialized!");
        localLop.telemetry.update();

    }
    public void initArm()
    {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(0.35);
        arm.setCurrentAlert(4, CurrentUnit.AMPS);

        while(!arm.isOverCurrent())
        {
            localLop.telemetry.addData("Current", arm.getCurrent(CurrentUnit.AMPS));
            localLop.telemetry.update();
        }

        arm.setPower(0);
        localLop.sleep(400);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        localLop.telemetry.addLine("Arm Initialized!");
        localLop.telemetry.update();
    }



    public void liftExtend()
    {
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(-1600);
        lift.setPower(1.0);
    }
    public void liftRetract()
    {
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(-1500);
        lift.setPower(1.0);
    }
    public void liftExtendManual()
    {
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift.setPower(1.0);
    }
    public void liftRetractManual()
    {
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift.setPower(-1.0);
    }
    public void liftStop()
    {
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(1.0);
    }


    public void elbowMove (int distance)
    {
        elbow.setPosition(elbow.getPosition() + distance);
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

    public void armExtend()
    {
        armPosition = ArmPosition.EXTEND;
        //elevatorTarget = 1895;
        elevatorTarget = -50;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armRetract()
    {
        armPosition = ArmPosition.RETRACT;
        //elevatorTarget = 177+35;
        elevatorTarget = -1819;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armHorizontal()
    {
        armPosition = ArmPosition.HORIZONTAL;
        //elevatorTarget = 1600;
        elevatorTarget = -300;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armVertical()
    {
        armPosition = ArmPosition.VERTICAL;
        //elevatorTarget = 625+30;
        elevatorTarget = -1250;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armPark()
    {
        armPosition = ArmPosition.HANG1;
        //elevatorTarget = 850+30;
        elevatorTarget = -1020;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armBack() {
        armPosition = ArmPosition.HOME;
       // elevatorTarget = 0;
        elevatorTarget = -1900;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }

    public void armHang2()
    {
        armPosition = ArmPosition.HANG2;
        elevatorTarget = -1575;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }

    public void elevatorDown()
    {
        elevatorTarget = 10;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        elevatorPosition = ElevatorPosition.DOWN;
    }
    public void elevatorHighBasket()
    {
        elevatorTarget = 2600;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        elevatorPosition = ElevatorPosition.HIGH_BASKET;
    }
    public void elevatorHighChamber()
    {
        elevatorTarget = 1400;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        elevatorPosition = ElevatorPosition.HIGH_CHAMBER;
    }
    public void elevatorHalf()
    {
        elevatorTarget = 650;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        elevatorPosition = ElevatorPosition.HIGH_CHAMBER;
    }

    public void elevatorMonitor()
    {
        if(elevator.isOverCurrent())
        {
            elevatorResetCounter += 1;
            firstPressed = false;
            elevator.setPower(0);
            elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        if((!elevatorLimit.isPressed()) && firstPressed && (elevatorPosition==ElevatorPosition.DOWN))
        {
            firstPressed = false;
            elevator.setPower(0);
        }
        if((elevatorLimit.isPressed()) && (!firstPressed))
        {
            firstPressed = true;
        }
    }

    public void elevatorOff()
    {
        elevator.setPower(0);
    }

    public void armOff() {
        arm.setPower(0);
    }


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

    public void tailUp()
    {
        tail.setPosition(-1);
    }

    public void tailDown()
    {
        tail.setPosition(1);
    }

    public void sampleDump()
        {
        dumper.setPosition(1.0);
    }

    public void samplePickup()
    {
        dumper.setPosition(-1.0);
    }

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

