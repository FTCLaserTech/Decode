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

public class DepotMain extends LinearOpMode
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

        Pose2d toInitialLaunchPosition = new Pose2d(autoFun.redBlueT(-25),20,Math.toRadians(autoFun.redBlueR(initialRotation,145)));
        Pose2d toSpike3 = new Pose2d(autoFun.redBlueT(-50),20,Math.toRadians(autoFun.redBlueR(initialRotation,0)));
        Pose2d pickupSpike3 = new Pose2d(autoFun.redBlueT(-50),-15,Math.toRadians(autoFun.redBlueR(initialRotation,0)));
        Pose2d toSpike2 = new Pose2d(autoFun.redBlueT(-75),20,Math.toRadians(autoFun.redBlueR(initialRotation,0)));
        Pose2d pickupSpike2 = new Pose2d(autoFun.redBlueT(-75),-21,Math.toRadians(autoFun.redBlueR(initialRotation,0)));

        // AFTER START IS PRESSED

        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        extras.setShooter(1350.0);

        //
        // shoot preload
        //
        // drive off the line and rotate towards the depot
        Action ToInitialPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(ToInitialPosition);

        // power up and aim the shooter
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        ExtraOpModeFunctions.TrackDepotState shooterReady = ExtraOpModeFunctions.TrackDepotState.NOTFOUND;
        while (!isStopRequested() && (shooterReady != ExtraOpModeFunctions.TrackDepotState.ONTARGET) && (timer.time() < 2))
        {
            shooterReady = extras.trackDepot();
            telemetry.update();
        }

        safeWaitSeconds(autoFun.startDelay);

        // shoot the artifacts
        safeWaitSeconds(1);
        extras.intakeForward();
        extras.ballStopOff();
        // wait for shooting to finish
        safeWaitSeconds(1.0);
        extras.ballStopOn();

        //
        // pickup and launch spike 3
        //
        Action GoToSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toSpike3.position, toSpike3.heading)
                .build();
        Actions.runBlocking(GoToSpike3);
        extras.intakeForward();
        Action PickupSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(pickupSpike3.position, pickupSpike3.heading)
                .build();
        Actions.runBlocking(PickupSpike3);
        extras.intakeOff();
        Action BackToLaunchSpot = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(BackToLaunchSpot);

        timer.reset();
        shooterReady = ExtraOpModeFunctions.TrackDepotState.NOTFOUND;
        while (!isStopRequested() && (shooterReady != ExtraOpModeFunctions.TrackDepotState.ONTARGET) && (timer.time() < 2))
        {
            shooterReady = extras.trackDepot();
            telemetry.update();
        }

        // shoot the artifacts
        //safeWaitSeconds(1);
        extras.intakeForward();
        extras.ballStopOff();
        // wait for shooting to finish
        safeWaitSeconds(1);
        extras.ballStopOn();

        //
        // pickup and launch spike 2
        //
        Action GoToSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toSpike2.position, toSpike2.heading)
                .build();
        Actions.runBlocking(GoToSpike2);
        extras.intakeForward();
        Action PickupSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(pickupSpike2.position, pickupSpike2.heading)
                .build();
        Actions.runBlocking(PickupSpike2);
        extras.intakeOff();
        Action BackToLaunchSpot2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(BackToLaunchSpot2);

        timer.reset();
        shooterReady = ExtraOpModeFunctions.TrackDepotState.NOTFOUND;
        while (!isStopRequested() && (shooterReady != ExtraOpModeFunctions.TrackDepotState.ONTARGET) && (timer.time() < 2))
        {
            shooterReady = extras.trackDepot();
            telemetry.update();
        }

        // shoot the artifacts
        //safeWaitSeconds(1);
        extras.intakeForward();
        extras.ballStopOff();
        // wait for shooting to finish
        safeWaitSeconds(1);
        extras.ballStopOn();

        safeWaitSeconds(2);

        // turn the intake and shooter off
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

