package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
//@Disabled

@Autonomous(group = "a")

public class Audience12Mixed extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 180;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        VisionFunctions vision = new VisionFunctions(hardwareMap, this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        AutoFunctions autoFun = new AutoFunctions(this, extras, vision);

        PinpointLocalizer ppLocalizer = (PinpointLocalizer) drive.localizer;
        double ppYawInitial = 0.0;
        double ppYawFinal = 0.0;
        double chYawInitial = 0.0;
        double chYawFinal = 0.0;
        double savedAngle = 0.0;

        drive.lazyImu.get().resetYaw();

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

            ppYawInitial = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);
            chYawInitial = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("ppYaw r: ", ppYawInitial);
            telemetry.addData("imuYaw r: ", chYawInitial);
            telemetry.addData("ppYaw d: ", Math.toDegrees(ppYawInitial));
            telemetry.addData("imuYaw d: ", Math.toDegrees(chYawInitial));
            telemetry.addData("savedAngle d: ", Math.toDegrees(savedAngle));

            telemetry.update();
        }

        // AFTER START IS PRESSED
        ElapsedTime timer = new ElapsedTime();

        Pose2d startPose = new Pose2d(-62, autoFun.redBlueT(-13.5), Math.toRadians(autoFun.redBlueT(initialRotation)));
        drive.localizer.setPose(startPose);
        Pose2d toInitialLaunchPosition = new Pose2d(-50,autoFun.redBlueT(-14),Math.toRadians(autoFun.redBlueT(155)));
        Pose2d toSpike1 = new Pose2d(-34,autoFun.redBlueT(-29),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d pickupSpike1 = new Pose2d(-36,autoFun.redBlueT(-50),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d toCorner = new Pose2d(-62,autoFun.redBlueT(-52),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d pickupCorner = new Pose2d(-62,autoFun.redBlueT(-60),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d toCorner2 = new Pose2d(-59,autoFun.redBlueT(-55),Math.toRadians(autoFun.redBlueT(0)));
        Pose2d pickupCorner2 = new Pose2d(-40,autoFun.redBlueT(-55),Math.toRadians(autoFun.redBlueT(0)));
        Pose2d toParkPosition = new Pose2d(-62,autoFun.redBlueT(-60),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d backToLaunchZone = new Pose2d(autoFun.redBlueT(0),0,Math.toRadians(270));

        extras.saveTeamColor(extras.teamColor);

        safeWaitSeconds(autoFun.startDelay);
        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        double launcherSpeed = 1800.0;
        extras.setLauncher(launcherSpeed);


        // drive off the line and rotate towards the depot
        Action ToLaunchPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        // launch ball sequence
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        ToLaunchPosition,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn()),
                        new InstantAction(() -> extras.intakeOff())
                        ),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch spike 1
        Action GoToSpike1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toSpike1.position, toSpike1.heading)
                .build();
        Actions.runBlocking(GoToSpike1);
        extras.intakeForward();
        Action PickupSpike1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(pickupSpike1.position, pickupSpike1.heading)
                .build();
        Actions.runBlocking(PickupSpike1);
        extras.intakeOff();

        Action ToLaunchPosition2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        ToLaunchPosition2,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch Corner 1
        extras.intakeForward();
        Action GoToCorner = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toCorner.position, toCorner.heading)
                .strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                .strafeToLinearHeading(toCorner.position, toCorner.heading)
                .strafeToLinearHeading(pickupCorner2.position, pickupCorner2.heading)
                //.strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new RaceAction(GoToCorner,extras.checkIntakeAction()));

        Action ToLaunchPosition3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new ParallelAction(ToLaunchPosition3,
                                new SequentialAction(new SleepAction(0.4),
                                new InstantAction(() -> extras.intakeOff()))),
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch Corner second time
        extras.intakeForward();
        Action GoToCorner2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toCorner.position, toCorner.heading)
                .strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                .strafeToLinearHeading(toCorner.position, toCorner2.heading)
                .strafeToLinearHeading(pickupCorner.position, pickupCorner2.heading)
                //.strafeToLinearHeading(toCorner.position, toCorner.heading)
                //.strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                //.strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new RaceAction(GoToCorner2,extras.checkIntakeAction()));
        extras.intakeOff();

        if(timer.seconds()>4)
        {
            Action ToLaunchPosition4 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                    .build();
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new ParallelAction(ToLaunchPosition4,
                                    new SequentialAction(new SleepAction(0.4),
                                            new InstantAction(() -> extras.intakeOff()))),
                            new InstantAction(() -> extras.intakeForward()),
                            new InstantAction(() -> extras.ballStopOff()),
                            new SleepAction(1.0),
                            new InstantAction(() -> extras.stopLauncher()),
                            new InstantAction(() -> extras.ballStopOn())),
                    extras.setLauncherAction(launcherSpeed)
            ));
        }

        // pickup and launch Corner second time
        extras.intakeForward();
        Action GoToCorner3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toCorner.position, toCorner.heading)
                .strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                //.strafeToLinearHeading(toCorner.position, toCorner.heading)
                //.strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                //.strafeToLinearHeading(toCorner.position, toCorner.heading)
                //.strafeToLinearHeading(pickupCorner.position, pickupCorner.heading)
                //.strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new RaceAction(GoToCorner3,extras.checkIntakeAction()));
        extras.intakeOff();

        Action ToLaunchPosition5 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new ParallelAction(ToLaunchPosition5,
                                new SequentialAction(new SleepAction(0.4),
                                        new InstantAction(() -> extras.intakeOff()))),
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));


        // Park
        Action toParkPosition1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toParkPosition.position, toParkPosition.heading)
                .build();
        extras.intakeForward();
        Action GoToCornerPark = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toParkPosition.position, toParkPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                        new ParallelAction(toParkPosition1,
                new InstantAction(() -> extras.stopLauncher()),
                new InstantAction(() -> extras.ballStopOn()),
                new InstantAction(() -> extras.intakeOff())

        //extras.setLauncherAction(launcherSpeed)
        )));

        safeWaitSeconds(2);

        // turn the intake and shooter off

        extras.intakeOff();
        extras.ballStopOn();
        //extras.setLauncher(0.0);

        // Save the ending location
        //extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
        //ppYawFinal = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);

        chYawFinal = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        savedAngle = chYawFinal - chYawInitial;
        extras.saveAutoStartRotation(savedAngle);
        PoseStorage.currentPose = drive.localizer.getPose();

        safeWaitSeconds(1.0);
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

