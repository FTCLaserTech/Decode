package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


@Config


public class AutoFunctions
{
    public LinearOpMode localLop = null;
    public ExtraOpModeFunctions localExtras = null;

    public double startDelay = 0.0;

    public AutoFunctions(LinearOpMode linearOpMode, ExtraOpModeFunctions extras) {
        localLop = linearOpMode;
        localExtras = extras;

        localExtras.setTurretMode(ExtraOpModeFunctions.TurretMode.ControlHub);
    }

    public void selectSide()
    {
        boolean colorNotSelected = true;
        localLop.telemetry.addLine("Select Team Color.");
        localLop.telemetry.addLine("Press X (blue) for Blue.");
        localLop.telemetry.addLine("Press B (red) for Red.");
        localLop.telemetry.update();
        while(colorNotSelected)
        {
            if(localLop.gamepad1.xWasPressed())
            {
                localExtras.teamColor = ExtraOpModeFunctions.TeamColor.BLUE;
                colorNotSelected = false;
            }
            if(localLop.gamepad1.bWasPressed())
            {
                localExtras.teamColor = ExtraOpModeFunctions.TeamColor.RED;
                colorNotSelected = false;
            }
        }
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
        if (localExtras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
        {
            localExtras.lights.setLightColor(ExtraOpModeFunctions.Lights.Light_Red);
            localExtras.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            localExtras.blinkinLedDriver.setPattern(localExtras.pattern);
        }
        else
        {
            localExtras.lights.setLightColor(ExtraOpModeFunctions.Lights.Light_Blue);
            localExtras.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            localExtras.blinkinLedDriver.setPattern(localExtras.pattern);
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

