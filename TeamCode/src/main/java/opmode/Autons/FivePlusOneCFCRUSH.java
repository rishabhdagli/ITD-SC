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

@Autonomous(name = "5 + 1")
@Config
public class FivePlusOneCFCRUSH extends LinearOpMode {
    public  static Follower follower;
    public boolean pathDone = false;
    public boolean prevPathDone = false, CountDone = false;
    Robot r;
    Boxtube boxtube;

    public static double posP1 = 24;
    public static double posP2 = 24.5;
    public static double posP3 = 26.5;
    public static double posP4 = 26.5;
    public static double posP5 = 26.5;

    public static double posScore = 38;

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
        follower.setStartingPose(new Pose(11.500,54.000, Math.toRadians(180)));
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
                        setPathState(PathStates.WallPickup);
                    }
                    break;

                case WallPickup:
                    if (!follower.isBusy()) {
                        CountDone = false;
                        resetActionTimer();

                        //Grabs specmin
                        //moves pivot back
                        if (actionTimer.time() < 0.2) {
                            r.SpecimenWallGrab();
                        }
                        else if (actionTimer.time() < 0.4) {
                            r.SpecimenWallUp();
                        }
                        else if (actionTimer.time() < 0.55) {
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
                                    setPathState(PathStates.Score2);
                                    break;
                            }
                            telemetry.addData("Path:", follower.getCurrentPath().toString());
                        }
                        else {
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
                                case 5:
                                    follower.followPath(Pickup5);
                                    setPathState(PathStates.SamplePickup);
                                    break;
                            }
                        }
                    }
                    break;

                case SamplePickup:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if (actionTimer.time() < 0.2) {
                            r.SpecimenWallGrab();
                        }
                        else if (actionTimer.time() < 0.4) {
                            r.SpecimenWallUp();
                        }
                        else if (actionTimer.time() < 0.55) {
                            r.PivotBack();
                        }
                        follower.followPath(ScoreSample);
                        setPathState(PathStates.SampleScore);
                    }
                case SampleScore:
                    r.PivotBack();
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.time() < 1){
                            r.BasketExtension();
                        } else if (actionTimer.time() < 1.3) {
                            r.BasketScore();
                        } else if (actionTimer.time() < 1.9) {
                            r.BasketReturn();
                        }

                    }
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
                                new Point(38.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        PrePush1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 73.000, Point.CARTESIAN),
                                new Point(2.065, 20.387, Point.CARTESIAN),
                                new Point(72.774, 44.645, Point.CARTESIAN),
                                new Point(70.968, 22.194, Point.CARTESIAN),
                                new Point(55.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0)
                .build();

        Push1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(55.000, 25.000, Point.CARTESIAN),
                                new Point(24.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndTimeoutConstraint(0)
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
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndTimeoutConstraint(0)
                .build();

        Push2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(24.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndTimeoutConstraint(0)
                .build();

        PrePush3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(24.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 18.000, Point.CARTESIAN),
                                new Point(57.000, 9.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(8)
                .setPathEndTimeoutConstraint(0)
                .build();

        Push3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 9.000, Point.CARTESIAN),
                                new Point(posP1, 9.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

//        Pickup1 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Point(35.000, 9.000, Point.CARTESIAN),
//                                new Point(posP1, 9.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setZeroPowerAccelerationMultiplier(4)
//                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posP1, 9.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(posScore, 69.2500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posScore, 69.250, Point.CARTESIAN),
                                new Point(16.774, 69.258, Point.CARTESIAN),
                                new Point(54.000, 33.500, Point.CARTESIAN),
                                new Point(posP2, 33.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posP2, 33.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(posScore, 71.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)

                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posScore, 71.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(54.000, 33.5, Point.CARTESIAN),
                                new Point(posP3, 33.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posP3, 33.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(posScore, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posScore, 70.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(54.000, 33.5, Point.CARTESIAN),
                                new Point(posP4, 33.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posP4, 33.500, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(posScore, 75.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();
        Pickup5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(posScore, 75.000, Point.CARTESIAN),
                                new Point(16.774, 72.258, Point.CARTESIAN),
                                new Point(54.000, 33.5, Point.CARTESIAN),
                                new Point(posP5, 33.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        ScoreSample = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(posP5, 33.500, Point.CARTESIAN),
                                new Point(12.129, 125.677, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
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