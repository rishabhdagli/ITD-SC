package opmode.Autons;


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

@Autonomous(name = "5 + 1 Spec Auto")
public class FivePlusOne extends LinearOpMode {
    public static Follower follower;
    public boolean pathDone = false;
    public boolean prevPathDone = false, CountDone = false;
    Robot r;
    Boxtube boxtube;
    EndEffector endEffector;
    int SpecimenCount = 0;
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
            Pickup5,
            Score1,
            Score2,
            Score3,
            Score4,
            Score5;
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
        follower = new Follower(hardwareMap);
        buildPaths();
        r = new Robot(hardwareMap, new Pose(7.000, 54.000));
        boxtube = r.boxtube;
        endEffector = r.endEffector;
        actionTimer.time(TimeUnit.SECONDS);

        while (opModeInInit()) {
//            boxtube.updatePiv();
//            boxtube.updateExt();
//            r.InitPosition();
        }

        waitForStart();

        currentPathState = PathStates.DriveToPreloadScoringPosition;
        actionTimer.reset();
        pathTimer.resetTimer();

        while (opModeIsActive()) {
            boxtube.updatePiv();
            boxtube.updateExt();
            follower.update();
            telemetry.addData("State", currentPathState);
            telemetry.update();

            switch (currentPathState) {

                case DriveToPreloadScoringPosition: //Driving to preload position
                    follower.followPath(Preload);
                    resetActionTimer();
//                    r.SpecimenPreLoad(); //moves pivot up
                    setPathState(PathStates.ScorePreload);
                    break;

                case ScorePreload: //Scoring preload
                    if (!follower.isBusy()) {
                        resetActionTimer();

                        if (actionTimer.time() < 1.0) {
//                            r.PreloadSpecExt(); //extends tube for preload
                        } else if (actionTimer.time() < 1.5) {
//                            r.SpecimenPreLoadScore();  //move pivot down
                            SpecimenCount = 1;
                        } else {
//                            r.SpecimenWallForSamplePush();
                            follower.followPath(PrePush1);
                            setPathState(PathStates.Push1);
                        }

                    }//case end
                    break;

                case Push1:
                    if (!follower.isBusy()) {
                        follower.followPath(Push1);
                        setPathState(PathStates.PrePush2);
                    }
                    break;

                case PrePush2:
                    if (!follower.isBusy()) {
                        follower.followPath(PrePush2);
                        setPathState(PathStates.Push2);
                    }
                    break;

                case Push2:
                    if (!follower.isBusy()) {
                        follower.followPath(Push2);
                        setPathState(PathStates.PrePush3);
                    }
                    break;

                case PrePush3:
                    if (!follower.isBusy()) {
                        follower.followPath(PrePush3);
                        setPathState(PathStates.Push3);
                    }
                    break;

                case Push3:
                    if (!follower.isBusy()) {
                        follower.followPath(Push3);
                        setPathState(PathStates.Pickup1);
                    }
                    break;

                case Pickup1:
                    if (!follower.isBusy()) {
                        follower.followPath(Pickup1);
                        setPathState(PathStates.WallPickup);
                    }
                    break;

                case WallPickup:
                    if (!follower.isBusy()) {
                        CountDone = false;
                        resetActionTimer();

                        //Grabs specmin
                        //moves pivot back
                        if (actionTimer.time() < 0.1) {
//                            r.SpecimenWallGrab();
                        } else if (actionTimer.time() < 0.15) {
//                            r.SpecimenWallUp();
                        } else if (actionTimer.time() < 0.65) {
                            switch (SpecimenCount) {
                                case 1:
                                    follower.followPath(Score1);
                                    break;
                                case 2:
                                    follower.followPath(Score2);
                                    break;
                                case 3:
                                    follower.followPath(Score3);
                                    break;
                                case 4:
                                    follower.followPath(Score4);
                                    break;
                                case 5:
                                    follower.followPath(Score5);
                            }
                        } else {
//                            r.SpecimenPreScore(); //Parrelel action
                            setPathState(PathStates.SCORING);
                        }

                    }
                    break;

                case SCORING:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (!CountDone) {
                            SpecimenCount++;
                            CountDone = true;
                        }

                        //brings the thing down
                        if (actionTimer.time() < 0.1) {
//                            r.SpecimenPostScore();
                        }
                        //drivetrain move
                        else if (actionTimer.time() < 1) {
                            switch (SpecimenCount) {
                                case 2:
                                    follower.followPath(Pickup2);
                                    break;
                                case 3:
                                    follower.followPath(Pickup3);
                                    break;
                                case 4:
                                    follower.followPath(Pickup4);
                                    break;
                                case 5:
                                    follower.followPath(Pickup5);

                            }
                        }
                        //gets ready for pick up
                        else {
//                            r.SpecimenWall();
                            setPathState(PathStates.WallPickup);
                        }

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
                                new Point(7.000, 54.000, Point.CARTESIAN),
                                new Point(25.000, 61.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        PrePush1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(25.000, 61.000, Point.CARTESIAN),
                                new Point(29.419, 22.452, Point.CARTESIAN),
                                new Point(58.323, 52.129, Point.CARTESIAN),
                                new Point(58.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        Push1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(58.000, 25.000, Point.CARTESIAN),
                                new Point(30.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(30.000, 25.000, Point.CARTESIAN),
                                new Point(59.613, 26.323, Point.CARTESIAN),
                                new Point(55.226, 27.613, Point.CARTESIAN),
                                new Point(57.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 16.000, Point.CARTESIAN),
                                new Point(30.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(30.000, 13.000, Point.CARTESIAN),
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 9.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 9.000, Point.CARTESIAN),
                                new Point(30.000, 9.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30.000, 9.000, Point.CARTESIAN),
                                new Point(19.500, 9.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(19.500, 9.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(43.25, 69.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(43.250, 69.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000, 35.000, Point.CARTESIAN),
                                new Point(27.000, 27.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(27.000, 27.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(43.250, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(43.250, 70.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000, 35.000, Point.CARTESIAN),
                                new Point(27.000, 27.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(27.000, 27.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(43.250, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(43.250, 73.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000, 35.000, Point.CARTESIAN),
                                new Point(27.000, 27.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.25)
                .build();

        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(27.000, 27.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(43.250, 77.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();

        Pickup5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(43.250, 77.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000, 35.000, Point.CARTESIAN),
                                new Point(10.219, 26.199, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.25)
                .build();

        Score5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(10.219, 26.199, Point.CARTESIAN),
                                new Point(10.405, 130.436, Point.CARTESIAN)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();


    }

    enum PathStates {
        DriveToPreloadScoringPosition, ScorePreload, PrePush2, PrePush3, Push1, Push2, Push3, Pickup1, SCORING, Pickup3, Pickup4, WallPickup, Score2, Score3, Score4, Park, End, WAIT1
    }
}