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

import Subsystems.Boxtube;
import Subsystems.EndEffector;
import Subsystems.Robot;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 Sample Paths only")
public class SamplePaths extends LinearOpMode {


    public static Follower follower;
    public Robot r;
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
        if (!prevPathDone) {
            actionTimer.resetTimer();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();
        r = new Robot(hardwareMap);
        follower.setStartingPose(new Pose(7.000,104.000,Math.toRadians(-90)));


        r.InitPosition();
        while(opModeInInit()){r.boxtube.update();}

        waitForStart();

        currentPathState = PathStates.Preload;
        actionTimer.resetTimer();
        pathTimer.resetTimer();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("State", currentPathState);
            telemetry.update();
            r.boxtube.update();
            switch (currentPathState) {

                case Preload:
                    r.PivotBack();
                    follower.followPath(PreloadScore);
                    setPathState(PathStates.PickupOne);
                    break;

                case PickupOne:
                    if (!follower.isBusy()) {
                        resetActionTimer();

                         if ( actionTimer.getElapsedTimeSeconds() < 2) {r.BasketExtension();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 4) {r.BasketScore();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() <6) {r.BasketReturn();
                         }
                         else if (actionTimer.getElapsedTimeSeconds() <8) {
                             follower.followPath(Pickup1);
                             setPathState(PathStates.ScoreOne);
                             r.SampleHoverAuto();
                         }



                    }
                    break;
                case ScoreOne:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 2){
                            follower.followPath(Score2);
                            setPathState(PathStates.PickupTwo);
                        }
                    }
                    break;

                case PickupTwo:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                         if(actionTimer.getElapsedTimeSeconds() > 2){
                            follower.followPath(Pickup2);
                            setPathState(PathStates.ScoreTwo);
                        }
                    }
                    break;

                case ScoreTwo:
                    if(!follower.isBusy()){
                    resetActionTimer();
                     if(actionTimer.getElapsedTimeSeconds() > 2){
                        follower.followPath(Score2);
                        setPathState(PathStates.PickupThree);
                    }
                    }
                    break;

                case PickupThree:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 2){
                            follower.followPath(Pickup3);
                            setPathState(PathStates.ScoreThree);
                        }
                    }
                    break;

                case ScoreThree:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() > 2){
                            follower.followPath(Score3);
                            setPathState(PathStates.Park);}
                    }
                    break;

                case Park:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() > 2){
                            follower.followPath(Park);
                        setPathState(PathStates.End);}
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
                                new Point(7.000,104.000,Point.CARTESIAN),
                                new Point(28.000,119.000,Point.CARTESIAN),
                                new Point(13.000,130.000,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-18))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(20.000, 126.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-18), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(23))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(20.000, 126.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(23), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(40))
                .setZeroPowerAccelerationMultiplier(4)
                .build();
//
        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(20.000, 126.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.500, 130.000, Point.CARTESIAN),
                                new Point(45.935, 128.516, Point.CARTESIAN),
                                new Point(67.355, 111.226, Point.CARTESIAN),
                                new Point(64.000, 93.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(4)
                .build();
    }
}


