package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
//@Disabled

@Autonomous(group = "a")

public class AudienceSimple extends LinearOpMode
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
            autoFun.autoInitFunction();
            safeWaitSeconds(0.01);

            telemetry.update();
        }

        // AFTER START IS PRESSED

        Pose2d toInitialLaunchPosition = new Pose2d(autoFun.redBlueT(10),0,Math.toRadians(autoFun.redBlueR(initialRotation,90)));
        Pose2d toParkPosition = new Pose2d(autoFun.redBlueT(0),-30,Math.toRadians(autoFun.redBlueR(initialRotation,0)));
        Pose2d toFirstArtifacts = new Pose2d(autoFun.redBlueT(30),-30,Math.toRadians(270));
        Pose2d pickUpFirstArtifacts = new Pose2d(autoFun.redBlueT(30),-40,Math.toRadians(270));
        Pose2d backToLaunchZone = new Pose2d(autoFun.redBlueT(0),0,Math.toRadians(270));

        extras.saveTeamColor(extras.teamColor);

        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        extras.setShooter(1800.0);

        // drive off the line and rotate towards the depot
        Action ToInitialPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(ToInitialPosition);

        // power up and aim the shooter
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        ExtraOpModeFunctions.TrackDepotState shooterReady = ExtraOpModeFunctions.TrackDepotState.NOTFOUND;
        while (!isStopRequested() && (shooterReady != ExtraOpModeFunctions.TrackDepotState.ONTARGET) && (timer.time() < 10))
        {
            shooterReady = extras.trackDepot();
            telemetry.update();
        }

        safeWaitSeconds(autoFun.startDelay);

        // shoot the artifacts if on target
        if (shooterReady == ExtraOpModeFunctions.TrackDepotState.ONTARGET)
        {
            safeWaitSeconds(3);
            extras.intakeForward();
            extras.ballStopOff();
            // wait for shooting to finish
            safeWaitSeconds(3);
        }

        // drive off the line
        Action ToPark = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toParkPosition.position, toParkPosition.heading)
                .build();
        Actions.runBlocking(ToPark);

        safeWaitSeconds(2);

        // turn the intake and shooter off
        extras.intakeOff();
        extras.ballStopOn();
        extras.setShooter(0.0);

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

