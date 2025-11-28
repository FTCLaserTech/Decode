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

        Pose2d toFirstArtifacts = new Pose2d(autoFun.redBlue(30),-30,Math.toRadians(270));
        Pose2d pickUpFirstArtifacts = new Pose2d(autoFun.redBlue(30),-40,Math.toRadians(270));
        Pose2d backToLaunchZone = new Pose2d(autoFun.redBlue(0),0,Math.toRadians(270));

        // AFTER START IS PRESSED

        // turn on the LimeLight
        vision.limelight.start();

        // power up and aim the shooter
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        ExtraOpModeFunctions.TrackDepotState shooterReady = ExtraOpModeFunctions.TrackDepotState.NOTFOUND;
        while (!isStopRequested() && (shooterReady != ExtraOpModeFunctions.TrackDepotState.ONTARGET) && (timer.time() < 5))
        {
            shooterReady = extras.trackDepot();
        }

        safeWaitSeconds(autoFun.startDelay);

        // shoot the artifacts if on target
        if (shooterReady == ExtraOpModeFunctions.TrackDepotState.ONTARGET)
        {
            extras.intakeForward();
            // wait for shooting to finish
            safeWaitSeconds(3);
        }

        // drive off the line
        Action PickupFirstArtifacts = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toFirstArtifacts.position, toFirstArtifacts.heading, new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-50,50))
                .strafeToLinearHeading(toFirstArtifacts.position, toFirstArtifacts.heading)
                .strafeToLinearHeading(pickUpFirstArtifacts.position, pickUpFirstArtifacts.heading)
                .strafeToLinearHeading(backToLaunchZone.position, backToLaunchZone.heading)
                .build();
        Actions.runBlocking(PickupFirstArtifacts);

        safeWaitSeconds(3);

        // turn the intake and shooter off
        extras.intakeOff();
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

