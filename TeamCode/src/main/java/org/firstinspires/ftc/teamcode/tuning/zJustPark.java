package org.firstinspires.ftc.teamcode.tuning;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoInit;
import org.firstinspires.ftc.teamcode.ExtraOpModeFunctions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.VisionFunctions;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
//@Disabled

@Autonomous(group = "a")

public class zJustPark extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d offLine = new Pose2d(0,10,Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        VisionFunctions vision = new VisionFunctions(hardwareMap, this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        AutoInit autoInit = new AutoInit(this, extras, vision);

        telemetry.addLine("Initialized");
        //telemetry.addData("x", drive.pose.position.x);
        //telemetry.addData("y", drive.pose.position.y);
        //telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        VisionFunctions.ObeliskPattern obelisk;

        AprilTagPoseFtc cam;
        AprilTagPoseFtc ll;

        while (!isStopRequested() && !opModeIsActive())
        {
            autoInit.autoInitFunction();
            safeWaitSeconds(0.01);

            telemetry.update();
        }

        // AFTER START IS PRESSED

        // turn on the LimeLight
        vision.limelight.start();

        safeWaitSeconds(autoInit.startDelay);

        // drive off the line
        Action DriveOfflineAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(offLine.position, offLine.heading)
                .build();
        Actions.runBlocking(DriveOfflineAction);

        safeWaitSeconds(3);

        // Save the ending location
        //extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
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

