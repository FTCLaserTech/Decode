package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


@Config


public class AutoFunctions
{
    public LinearOpMode localLop = null;
    public VisionFunctions localVision = null;
    public ExtraOpModeFunctions localExtras = null;

    public double startDelay = 0.0;

    public AutoFunctions(LinearOpMode linearOpMode, ExtraOpModeFunctions extras, VisionFunctions vision) {
        localLop = linearOpMode;
        localVision = vision;
        localExtras = extras;
    }

    public void autoInitFunction()
    {
        // use g1 x button to change red/blue
        if(localLop.gamepad1.xWasPressed())
        {
            if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
            {
                localExtras.teamColor = ExtraOpModeFunctions.TeamColor.BLUE;
            }
            else
            {
                localExtras.teamColor = ExtraOpModeFunctions.TeamColor.RED;
            }

            localExtras.saveTeamColor(localExtras.teamColor);
        }
        localLop.telemetry.addData("Team Color - gp1 x: ", localExtras.teamColor);
        if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
        {
            localExtras.light1.setPosition(localExtras.Light_Red);
            localExtras.light2.setPosition(localExtras.Light_Red);
        }
        else
        {
            localExtras.light1.setPosition(localExtras.Light_Blue);
            localExtras.light2.setPosition(localExtras.Light_Blue);
        }

        // load artifacts
        localLop.telemetry.addLine("Load Artifacts - gp1 Left Trigger: ");
        if (localLop.gamepad1.left_trigger > 0)
        {
            localExtras.ballStopOn();
            localExtras.intakeForward();
        }
        else
        {
            localExtras.intakeOff();
        }

        // use gp1 dpad up down buttons to add a delay after start
        if(localLop.gamepad1.dpadUpWasPressed())
        {
            startDelay += 0.5;
        }
        if(localLop.gamepad1.dpadDownWasPressed())
        {
            startDelay -= 0.5;
            startDelay = Math.max(0,startDelay);
        }
        localLop.telemetry.addData("Start Delay - gp1 dup, ddn: ", startDelay);

        // use g1 a button to check obelisk and point at depot
        if(localLop.gamepad1.a)
        {
            // check the obelisk
            if(!localVision.limelight.isRunning())
            {
                localVision.limelight.start();
            }
            VisionFunctions.ObeliskPattern obelisk = localVision.readObeliskLimelight();
            localLop.telemetry.addData("Obelisk - gp1 a: ", obelisk);

            // check the depot
            AprilTagPoseFtc pose;
            if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
            {
                pose = localVision.readRedAprilTag_ll();
            }
            else
            {
                pose = localVision.readBlueAprilTag_ll();
            }
            if(pose != null)
            {
                // move to the target
                localLop.telemetry.addData("Depot angle: ", pose.bearing);
                localLop.telemetry.addData("Depot range: ", pose.range);
                //localExtras.trackDepot();
            }
            else
            {
                // report target not found
                if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                {
                    localLop.telemetry.addLine("Red Depot Not Found");
                }
                else
                {
                    localLop.telemetry.addLine("Blue Depot Not Found");
                }
            }
        }
        else
        {
            if(localVision.limelight.isRunning())
            {
                localVision.limelight.stop();
                localExtras.turretCR.setPower(0.0);
            }
            localLop.telemetry.addLine("Vision Check - g1 a");
        }
    }

    public double redBlueT(double value)
    {
        if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.BLUE)
        {
            return(-value);
        }
        else
        {
            return(value);
        }
    }

    public double redBlueR(double start, double value)
    {
        if(localExtras.teamColor == ExtraOpModeFunctions.TeamColor.BLUE)
        {
            return(start+value);
        }
        else
        {
            return(start-value);
        }
    }
}

