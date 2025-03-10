package opmode.Autons;

import android.widget.GridLayout;

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

@Autonomous(name = "5 Specimen")
public class FiveSpecCFCRUSH extends LinearOpMode {
    public  static Follower follower;
    public boolean pathDone = false;
    public boolean prevPathDone = false, CountDone = false;
    Robot r;
    Boxtube boxtube;
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
        follower.setStartingPose(new Pose(7.000,57.000, Math.toRadians(90)));
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

        currentPathState = PathStates.DriveToPreloadScoringPosition;
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

                case DriveToPreloadScoringPosition: //Drivng to preload position
                    follower.followPath(Preload);
                    resetActionTimer();
                    r.SpecimenPreScore(); //moves pivot up
                    setPathState(PathStates.ScorePreload);
                    break;

                case ScorePreload: //Scoring preload
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        r.SpecimenPostScore();
                        follower.followPath(PrePush1);
                        setPathState(PathStates.Push1);
                        }
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
                        r.SpecimenWall();
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
                            r.SpecimenWallGrab();
                        } else if (actionTimer.time() < 0.15) {
                            r.SpecimenWallUp();
                        } else if (actionTimer.time() < 0.35) {
                            r.SpecimenPreScore();
                            switch (SpeciminCount) {
                                case 1:
                                    follower.followPath(Score1);
                                    setPathState(PathStates.SCORING);
                                    break;
                                case 2:
                                    follower.followPath(Score2);
                                    setPathState(PathStates.SCORING);
                                    break;
                                case 3:
                                    follower.followPath(Score3);
                                    setPathState(PathStates.SCORING);
                                    break;
                                case 4:
                                    follower.followPath(Score4);
                                    setPathState(PathStates.SCORING);
                                    break;
                            }
                            telemetry.addData("Path:", follower.getCurrentPath().toString());
                        } else {
                            r.SpecimenPreScore(); //Parrelel action
                        }

                    }
                    break;

                case SCORING:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (!CountDone) {
                            SpeciminCount++;
                            CountDone = true;
                        }

                        //brings the thing down
                        if (actionTimer.time() < 0.5) {
                            r.SpecimenPostScore();
                        }
                        //drivetrain move
                        else if (actionTimer.time() < 1) {
                            r.SpecimenWall();
                            switch (SpeciminCount) {
                                case 2:
                                    follower.followPath(Pickup2);
                                    setPathState(PathStates.WallPickup);
                                    break;
                                case 3:
                                    follower.followPath(Pickup3);
                                    setPathState(PathStates.WallPickup);
                                    break;
                                case 4:
                                    follower.followPath(Pickup4);
                                    setPathState(PathStates.WallPickup);
                                    break;

                            }
                        }
                        //gets ready for pick up
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
                                new Point(7.000, 57.000, Point.CARTESIAN),
                                new Point(40.00, 73.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        PrePush1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(40.000, 70.000, Point.CARTESIAN),
                                new Point(2.065, 20.387, Point.CARTESIAN),
                                new Point(72.774, 44.645, Point.CARTESIAN),
                                new Point(70.968, 22.194, Point.CARTESIAN),
                                new Point(55.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(55.000, 26.000, Point.CARTESIAN),
                                new Point(26.000, 28.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(26.000, 26.000, Point.CARTESIAN),
                                new Point(59.613, 26.323, Point.CARTESIAN),
                                new Point(55.226, 27.613, Point.CARTESIAN),
                                new Point(57.000, 20.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 20.000, Point.CARTESIAN),
                                new Point(26.000, 20.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(30.000, 20.000, Point.CARTESIAN),
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 15.000, Point.CARTESIAN),
                                new Point(30.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30.000, 15.000, Point.CARTESIAN),
                                new Point(20.5000, 15.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(20.500, 13.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(38.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 73.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(31.484, 39.742, Point.CARTESIAN),
                                new Point(20.500, 30.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(20.500, 30.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(38.000, 76.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 76.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(31.484, 39.742, Point.CARTESIAN),
                                new Point(20.5000, 30.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(20.5000, 30.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(38.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 73.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(31.484, 39.742, Point.CARTESIAN),
                                new Point(20.5000, 30.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(20.5000, 30.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(38.000, 77.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
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
        DriveToPreloadScoringPosition, ScorePreload, PrePush2, PrePush3, Push1, Push2, Push3, Pickup1, SCORING, Pickup3, Pickup4, WallPickup, Score2, Score3, Score4, Park, End, WAIT1
    }
}