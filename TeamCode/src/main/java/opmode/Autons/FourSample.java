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

import config.auto.AutoRobot;
import config.Subsystem.Boxtube;
import config.Subsystem.EndEffector;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 Sample")
public class FourSample extends LinearOpMode {
    AutoRobot r;

    Boxtube boxtube;

    EndEffector endEffector;

    public static Follower follower;
    private Timer pathTimer = new Timer();
    private Timer actionTimer = new Timer();
    public boolean pathDone = false;
    public boolean prevPathDone = false;

    PathChain PreloadScore,
            Pickup1,
            Score1,
            Pickup2,
            Score2,
            Pickup3,
            Score3,
            Park;

    enum PathStates {
        Preload, PickupOne, ScoreOne, PickupTwo, ScoreTwo, PickupThree, ScoreThree, Park, End
    }

    PathStates currentPathState;

    private void setPathState(PathStates state) {
        currentPathState = state;
        pathTimer.resetTimer();
        pathDone = false;
    }

    private void resetActionTimer() {
        pathDone = true;
        if (pathDone && !prevPathDone) {
            actionTimer.resetTimer();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();
        r = new AutoRobot(hardwareMap, telemetry, new Pose(7.000, 114.000));
        boxtube = r.boxtube;
        endEffector = r.endEffector;
        r.t.addData("init", true);
        r.t.update();

//        while(opModeInInit()){
//            boxtube.updatePiv();
//            boxtube.updateExt();
//            r.InitPosition();
//        }

        waitForStart();

        currentPathState = PathStates.Preload;
        actionTimer.resetTimer();
        pathTimer.resetTimer();

        while (opModeIsActive()) {
            boxtube.updatePiv();
            boxtube.updateExt();
            follower.update();
            telemetry.addData("State", currentPathState);
            telemetry.update();

            switch (currentPathState) {

                case Preload:
                    follower.followPath(PreloadScore);
                    setPathState(PathStates.PickupOne);
                    break;

                case PickupOne:
                    if(!follower.isBusy()){


                            follower.followPath(Pickup1);
                            setPathState(PathStates.ScoreOne);


//                        resetActionTimer();

//                        r.PivotBack();

//                        if(actionTimer.getElapsedTimeSeconds() > 1.5){
//                            r.BasketExtension();
//                            if(actionTimer.getElapsedTimeSeconds() > 2.5){
//                                r.ClawOpen();
//                                if(actionTimer.getElapsedTimeSeconds() > 3){
//                                    r.BasketReturn();
//                                    if(actionTimer.getElapsedTimeSeconds() > 3.5){
//                                        r.Loiter();
//                                        follower.followPath(Pickup1);
//                                        setPathState(PathStates.ScoreOne);
//                                    }
//                                }
//                            }
//                        }

                    }
                    break;

                case ScoreOne:
                    if(!follower.isBusy()) {




                            follower.followPath(Score1);
                            setPathState(PathStates.PickupTwo);


//                        resetActionTimer();
//
//                        if(actionTimer.getElapsedTimeSeconds() > 1){
//                            r.SampleHover();
//                            if(actionTimer.getElapsedTimeSeconds() > 1.5){
//                                r.SampleGrab();
//                                if(actionTimer.getElapsedTimeSeconds() > 2){
//                                    r.LoiterSample();
//                                    if(actionTimer.getElapsedTimeSeconds() > 2.5){
//                                        follower.followPath(Score1);
//                                        setPathState(PathStates.PickupTwo);
//                                    }
//                                }
//                            }
//                        } else {
//                            r.Loiter();
//                        }

                    }
                    break;

                case PickupTwo:
                    if(!follower.isBusy()) {


                            follower.followPath(Pickup2);
                            setPathState(PathStates.ScoreTwo);

//                        if(actionTimer.getElapsedTimeSeconds() > 1){
//                            r.PivotBack();
//                            if(actionTimer.getElapsedTimeSeconds() > 2){
//                                r.BasketExtension();
//                                if(actionTimer.getElapsedTimeSeconds() > 3){
//                                    r.ClawOpen();
//                                    if(actionTimer.getElapsedTimeSeconds() > 3.5){
//                                        r.BasketReturn();
//                                        if(actionTimer.getElapsedTimeSeconds() > 4) {
//                                            r.Loiter();
//                                            follower.followPath(Pickup2);
//                                            setPathState(PathStates.ScoreTwo);
//                                        }
//                                    }
//                                }
//                            }
//                        }


                    }
                    break;

                case ScoreTwo:
                    if(!follower.isBusy()) {
                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds() > 1){
                            follower.followPath(Score2);
                            setPathState(PathStates.PickupThree);
                        }

                    }
                    break;

                case PickupThree:
                    if(!follower.isBusy()) {
                        follower.followPath(Pickup3);
                        setPathState(PathStates.ScoreThree);
                    }
                    break;

                case ScoreThree:
                    if(!follower.isBusy()) {
                        follower.followPath(Score3);
                        setPathState(PathStates.Park);
                    }
                    break;

                case Park:
                    if(!follower.isBusy()) {
                        follower.followPath(Park);
                        setPathState(PathStates.End);
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

        PreloadScore = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.000, 102.000, Point.CARTESIAN),
                                new Point(20.000, 115.000, Point.CARTESIAN),
                                new Point(15.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.000, 135.000, Point.CARTESIAN),
                                new Point(25.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(25.000, 130.000, Point.CARTESIAN),
                                new Point(15.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.000, 135.000, Point.CARTESIAN),
                                new Point(25.000, 145.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(25.000, 145.000, Point.CARTESIAN),
                                new Point(15.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(25.000, 135.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(15))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(25.000, 135.000, Point.CARTESIAN),
                                new Point(17.000, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(17.000, 125.000, Point.CARTESIAN),
                                new Point(65.241, 112.457, Point.CARTESIAN),
                                new Point(60.000, 94.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();
    }
}


