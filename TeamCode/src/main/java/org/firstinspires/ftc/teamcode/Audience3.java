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

public class Audience3 extends LinearOpMode
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
            telemetry.addData("chYaw r: ", chYawInitial);
            telemetry.addData("ppYaw d: ", Math.toDegrees(ppYawInitial));
            telemetry.addData("chYaw d: ", Math.toDegrees(chYawInitial));

            telemetry.update();
        }

        // AFTER START IS PRESSED

        Pose2d startPose = new Pose2d(-62, autoFun.redBlueT(-13.5), Math.toRadians(autoFun.redBlueT(initialRotation)));
        drive.localizer.setPose(startPose);
        Pose2d toInitialLaunchPosition = new Pose2d(-50,autoFun.redBlueT(-14),Math.toRadians(autoFun.redBlueT(155)));
        Pose2d toParkPosition = new Pose2d(-60,autoFun.redBlueT(-35),Math.toRadians(autoFun.redBlueT(270)));
        Pose2d toFirstArtifacts = new Pose2d(autoFun.redBlueT(30),-30,Math.toRadians(270));
        Pose2d pickUpFirstArtifacts = new Pose2d(autoFun.redBlueT(30),-40,Math.toRadians(270));
        Pose2d backToLaunchZone = new Pose2d(autoFun.redBlueT(0),0,Math.toRadians(270));

        extras.saveTeamColor(extras.teamColor);

        safeWaitSeconds(autoFun.startDelay);
        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        double launcherSpeed = 1800.0;
        extras.setLauncher(launcherSpeed);

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

        // drive off the line
        Action ToPark = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toParkPosition.position, toParkPosition.heading)
                .build();
        Actions.runBlocking(ToPark);

        // turn the intake and shooter off
        extras.intakeOff();
        extras.ballStopOn();
        extras.launcher1.setPower(0.0);
        extras.launcher2.setPower(0.0);
        //extras.setLauncher(0.0);

        // Save the ending location
        //savedAngle = chYawInitial;

        chYawFinal = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        savedAngle = chYawFinal - chYawInitial;
        //extras.saveAutoStartRotation(savedAngle);
        PoseStorage.currentAngle = savedAngle;
        PoseStorage.currentPose = drive.localizer.getPose();

        safeWaitSeconds(1);
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

