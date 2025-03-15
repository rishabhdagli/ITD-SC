package opmode.Autons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import Subsystems.Boxtube;
import Subsystems.EndEffector;
import Subsystems.Robot;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = " + 1 testing")
@Config
public class plusOneTester extends LinearOpMode {
    public  static Follower follower;
    public boolean pathDone = false;
    public boolean dpadEdger = false;
    public boolean prevPathDone = false, CountDone = false;
    Robot r;
    Boxtube boxtube;

    public static double pickupX = 23,pickupY = 33,
            IncrementScoreY = 2.0,ScoreX = 40, ScoreY = 67, preloadX = 40, sampleScoreY = 124, sampleScoreX = 13,
            ControlPickupX = 37,ControlPickupY = 36,ControlScoreX = 12.903,ControlScoreY = 62.000;
    public static double sampleScoreFinalHeading = 300;
    public static double pushingZpam = 8, pickup1Zpam = 2, pickup3Zpam = 2, pickup4Zpam = 2, pickup5Zpam = 2, score1Zpam = 4, score2Zpam = 4, score3Zpam = 4, score4Zpam = 4, push3Zpam = 4, preloadZpam = 4, incrementPickup1X = -5, sampleScoreZpam = 12;
    public static double pushingTimeout = 0, pickup1Timeout = 500,pickup3Timeout = 0, pickup4Timeout = 0, pickup5Timeout = 0, score1Timeout = 50, score2Timeout = 50, score3Timeout = 50, score4Timeout = 50, push3Timeout = 300, sampleScoreTimeout = 0;

    EndEffector endEffector;
    int SpeciminCount = 1;
    PathChain Preload,
            PrePush1,
            PrePush2,
            PrePush3,
            Push1,
            Push2,
            Push3,
            Pickup1,
            Pickup2,
            Pickup3,
            Pickup4,
            Score1,
            Score2,
            Score3,
            Score4,
            Pickup5,
            ScoreSample,
            Park;
    PathStates currentPathState, lastPathState;
    private Timer pathTimer = new Timer();
    private ElapsedTime actionTimer = new ElapsedTime();

    private void setPathState(PathStates state) {
        currentPathState = state;
        pathTimer.resetTimer();
        pathDone = false;
    }

    private void resetActionTimer() {
        pathDone = true;
        if (pathDone && !prevPathDone) {
            actionTimer.reset();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);

        r = new Robot(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(23.000,33.000, Math.toRadians(180)));
        buildPaths();

        boxtube = r.boxtube;
        endEffector = r.endEffector;
        actionTimer.time(TimeUnit.SECONDS);

        r.InitPosition();
        while (opModeInInit()) {
            r.boxtube.update();
            follower.update();
        }

        waitForStart();

        currentPathState = PathStates.SamplePickup;
        actionTimer.reset();
        pathTimer.resetTimer();

        while (opModeIsActive()) {
            boxtube.updatePiv();
            boxtube.updateExt();
            follower.update();
            telemetry.addData("State", currentPathState);
            telemetry.addData("Specimen Count", SpeciminCount);
            telemetry.update();

            switch (currentPathState) {
                case SamplePickup:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if (actionTimer.time() < 0.2) {
                            r.SpecimenWallGrab();
                        }
                        else if (actionTimer.time() < 0.4) {
                            r.SpecimenWallUpTele();
                        }
                        else {
                            follower.followPath(ScoreSample);
                            setPathState(PathStates.SampleScore);
                        }
                    }
                    break;
                case SampleScore:
                    r.PivotBack();
                    if(boxtube.Pivot.getCurrentPosition() < -900){r.BasketExtension();}
                    if(!follower.isBusy()){
                        r.BasketScore();
                    }
                    break;
                case End:
                default:
                    if (!follower.isBusy()) {
                        return;
                    }
            }
            prevPathDone = pathDone;
        }
    }

    public void buildPaths() {

        Preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(11.500, 57.000, Point.CARTESIAN),
                                new Point(preloadX, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(preloadZpam)
                .build();

        PrePush1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.000, 73.000, Point.CARTESIAN),
                                new Point(2.065, 20.387, Point.CARTESIAN),
                                new Point(72.774, 44.645, Point.CARTESIAN),
                                new Point(70.968, 22.194, Point.CARTESIAN),
                                new Point(55.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(pushingTimeout)
                .build();

        Push1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(55.000, 25.000, Point.CARTESIAN),
                                new Point(24.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pushingZpam)
                .setPathEndTimeoutConstraint(pushingTimeout)
                .build();

        PrePush2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(24.000, 25.000, Point.CARTESIAN),
                                new Point(59.613, 26.323, Point.CARTESIAN),
                                new Point(55.226, 27.613, Point.CARTESIAN),
                                new Point(57.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pushingZpam)
                .setPathEndTimeoutConstraint(pushingTimeout)
                .build();

        Push2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(24.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pushingZpam)
                .setPathEndTimeoutConstraint(pushingTimeout)
                .build();

        PrePush3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(24.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 10.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pushingZpam)
                .setPathEndTimeoutConstraint(pushingTimeout)
                .build();

        Push3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 10.000, Point.CARTESIAN),
                                new Point((pickupX+incrementPickup1X), 10.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(push3Zpam)
                .setPathEndTimeoutConstraint(push3Timeout)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point((pickupX+incrementPickup1X), 9.500, Point.CARTESIAN),
                                new Point(ControlScoreX, ControlScoreY, Point.CARTESIAN),
                                new Point(ScoreX, ScoreY + IncrementScoreY*4, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(score1Zpam)
                .setPathEndTimeoutConstraint(score1Timeout)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(ScoreX, ScoreY + IncrementScoreY*4, Point.CARTESIAN),
                                new Point(ControlPickupX, ControlPickupY, Point.CARTESIAN),
                                new Point((pickupX-2), pickupY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pickup1Zpam)
                .setPathEndTimeoutConstraint(pickup1Timeout)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(ControlScoreX, ControlScoreY, Point.CARTESIAN),
                                new Point(ScoreX, ScoreY + IncrementScoreY*3, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(score2Zpam)
                .setPathEndTimeoutConstraint(score2Timeout)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(ScoreX, ScoreY + IncrementScoreY*3, Point.CARTESIAN),
                                new Point(ControlPickupX, ControlPickupY, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pickup3Zpam)
                .setPathEndTimeoutConstraint(pickup3Timeout)
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(ControlScoreX, ControlScoreY, Point.CARTESIAN),
                                new Point(ScoreX, ScoreY + IncrementScoreY*2, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(score3Zpam)
                .setPathEndTimeoutConstraint(score3Timeout)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(ScoreX, ScoreY + IncrementScoreY*2, Point.CARTESIAN),
                                new Point(ControlPickupX, ControlPickupY, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pickup4Zpam)
                .setPathEndTimeoutConstraint(pickup4Timeout)
                .build();

        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(ControlScoreX, ControlScoreY, Point.CARTESIAN),
                                new Point(ScoreX, ScoreY + IncrementScoreY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(score4Zpam)
                .setPathEndTimeoutConstraint(score4Timeout)
                .build();
        Pickup5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(ScoreX, ScoreY + IncrementScoreY, Point.CARTESIAN),
                                new Point(ControlPickupX, ControlPickupY, Point.CARTESIAN),
                                new Point(pickupX, pickupY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(pickup5Zpam)
                .setPathEndTimeoutConstraint(pickup5Timeout)
                .build();
        ScoreSample = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickupX, pickupY, Point.CARTESIAN),
                                new Point(sampleScoreX, sampleScoreY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(sampleScoreFinalHeading))
                .setZeroPowerAccelerationMultiplier(sampleScoreZpam)
                .setPathEndTimeoutConstraint(sampleScoreTimeout)
                .build();
        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(43.250, 77.000, Point.CARTESIAN),
                                new Point(22.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    enum PathStates {
        Pickup5,SamplePickup,SampleScore, DriveToPreloadScoringPosition, ScorePreload, PrePush2, PrePush3, Push1, Push2, Push3, Pickup1, SCORING, Pickup3, Pickup4, WallPickup, Score2, Score3, Score4, Park, End, WAIT1
    }
}

