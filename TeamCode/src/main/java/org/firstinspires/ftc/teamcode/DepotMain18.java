package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
@Disabled

@Autonomous(group = "a")

public class DepotMain18 extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 180;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        safeWaitSeconds(0.3);
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


            telemetry.update();
        }

        // AFTER START IS PRESSED
        extras.setTurret(Math.toRadians(autoFun.redBlueT(-125)));

        Pose2d startPose = new Pose2d(61, autoFun.redBlueT(-37), Math.toRadians(autoFun.redBlueT(initialRotation)));
        drive.localizer.setPose(startPose);
        Pose2d toInitialLaunchPosition = new Pose2d(12,autoFun.redBlueT(-24),Math.toRadians(autoFun.redBlueT(initialRotation))); // old position(12,-17)
        Pose2d toSpike3 = new Pose2d(11,autoFun.redBlueT(-28),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike3 = new Pose2d(11,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation))); //-50
        Pose2d nearGate = new Pose2d(0.5,autoFun.redBlueT(-35),Math.toRadians(autoFun.redBlueT(initialRotation))); //-31
        Pose2d toGate = new Pose2d(0.5,autoFun.redBlueT(-55),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d nearGate2 = new Pose2d(-11,autoFun.redBlueT(-56),Math.toRadians(autoFun.redBlueT(305))); //-31
        Pose2d toGate2 = new Pose2d(-11,autoFun.redBlueT(-57),Math.toRadians(autoFun.redBlueT(310)));
        Pose2d toSpike2 = new Pose2d(-11,autoFun.redBlueT(-27),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike2 = new Pose2d(-11,autoFun.redBlueT(-47),Math.toRadians(autoFun.redBlueT(initialRotation))); //-50
        Pose2d toSpike1 = new Pose2d(-35,autoFun.redBlueT(-30),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike1 = new Pose2d(-35,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation))); //-51
        Pose2d park = new Pose2d(-1,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation)));

        extras.saveTeamColor(extras.teamColor);

        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        double launcherSpeed = 1350.0;
        extras.setLauncher(launcherSpeed);
        extras.launcherSdown();


        //safeWaitSeconds(autoFun.startDelay);

        //
        // shoot preload
        //
        // drive off the line and rotate towards the depot
        Action ToInitialPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        ToInitialPosition,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
                ));

//
        // pickup and launch spike 2
        //
        Action GoToSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike2.position, toSpike2.heading)
                .splineToConstantHeading(pickupSpike2.position, pickupSpike2.heading)
                .splineToConstantHeading(toSpike2.position, toSpike2.heading)
                //.strafeToLinearHeading(toSpike2.position, toSpike2.heading)
                .build();
        Actions.runBlocking(GoToSpike2);
        extras.intakeForward();
        Action BackToLaunchSpot2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        BackToLaunchSpot2,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));


        Action Togate2 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike2.position, toSpike2.heading)
                .strafeToSplineHeading(nearGate2.position, nearGate2.heading)
                .splineToLinearHeading(toGate2,toGate2.heading)
                //.strafeToLinearHeading(nearGate.position, nearGate.heading)
                .build();
        Actions.runBlocking(Togate2);
        extras.intakeForward();
        safeWaitSeconds(1);
        Action BackToLaunchSpot3 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(toSpike2,toSpike2.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        extras.intakeOff();


        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        BackToLaunchSpot3,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));

        Action Togate = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike2.position, toSpike2.heading)
                .strafeToSplineHeading(nearGate2.position, nearGate2.heading)
                .splineToLinearHeading(toGate2,toGate2.heading)
                //.strafeToLinearHeading(nearGate.position, nearGate.heading)
                .build();
        Actions.runBlocking(Togate);
        extras.intakeForward();
        safeWaitSeconds(1);

        Action BackToLaunchSpot5 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(toSpike2,toSpike2.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        extras.intakeOff();


        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        BackToLaunchSpot5,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));
        //
        // pickup spike 3, open the gate, launch spike 3
        //
        extras.intakeForward();
        Action GoToSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike3.position, toSpike3.heading)
                .splineToConstantHeading(pickupSpike3.position, pickupSpike3.heading)
                //.strafeToSplineHeading(toSpike3.position, toSpike3.heading)
                //.strafeToSplineHeading(pickupSpike3.position, pickupSpike3.heading)
                //.strafeToLinearHeading(toSpike3.position, toSpike3.heading)
                .build();
        Actions.runBlocking(GoToSpike3);

        Action BackToLaunchSpot4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        extras.intakeOff();


        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        BackToLaunchSpot4,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch spike 1
        extras.intakeForward();
        Action GoToSpike1 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike1.position, toSpike1.heading)
                .splineToConstantHeading(pickupSpike1.position, pickupSpike1.heading)
                //.strafeToLinearHeading(toSpike1.position, toSpike1.heading)
                .build();
        Actions.runBlocking(GoToSpike1);

        Action BackToLaunchSpot6 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        BackToLaunchSpot6,
                        new InstantAction(() -> extras.intakeForward()),
                        new InstantAction(() -> extras.ballStopOff()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.ballStopOn())),
                extras.setLauncherAction(launcherSpeed)
        ));

        extras.intakeOff();

        Action Park = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(park.position, park.heading)
                .build();
        Actions.runBlocking(Park);

        safeWaitSeconds(1);

        // turn the intake and shooter off
        extras.ballStopOn();
        //extras.setLauncher(0.0);
        extras.launcher1.setPower(0.0);
        extras.launcher2.setPower(0.0);

        // Save the ending location
        //extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
        ppYawFinal = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);

        chYawFinal = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        savedAngle = chYawFinal - chYawInitial;
        //extras.saveAutoStartRotation(savedAngle);
        PoseStorage.currentAngle = savedAngle;
        PoseStorage.currentPose = drive.localizer.getPose();

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

