package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
//@Disabled

@Autonomous(group = "a")

public class zJustParkDepot extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        VisionFunctions vision = new VisionFunctions(hardwareMap, this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        AutoFunctions autoFun = new AutoFunctions(this, extras, vision);

        extras.teamColor = extras.readTeamColor();

        telemetry.addLine("Initialized");
        //telemetry.addData("x", drive.pose.position.x);
        //telemetry.addData("y", drive.pose.position.y);
        //telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        VisionFunctions.ObeliskPattern obelisk;
        AprilTagPoseFtc ll;
        while (!isStopRequested() && !opModeIsActive())
        {
            autoFun.autoInitFunction();
            safeWaitSeconds(0.01);

            telemetry.update();
        }

        Pose2d offLine = new Pose2d(autoFun.redBlueT(-2),20,Math.toRadians(270));

        // AFTER START IS PRESSED

        // turn on the LimeLight
        vision.limelight.start();

        safeWaitSeconds(autoFun.startDelay);

        // drive off the line
        Action DriveOfflineAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(offLine.position, offLine.heading)
                .build();
        Actions.runBlocking(DriveOfflineAction);

        safeWaitSeconds(3);

        // Save the ending location
        extras.saveAutoStartRotation(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        // Save the team color
        extras.saveTeamColor(extras.teamColor);
    }

    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time)
        {
            ;
        }
    }
}

