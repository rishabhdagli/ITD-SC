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

import ActionScheduler.Event;
import ActionScheduler.Task;
import Subsystems.AutoRobot;
import Subsystems.Boxtube;
import Subsystems.EndEffector;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 Sample Trial Schduled")
public class FourSampleSchduled extends LinearOpMode {

    Event PickupOne;


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
        PickupOne = new Event();


        // Task 2: At 1.5 seconds, run BasketExtension().
        PickupOne.addTimedAction(new Task() {
            @Override
            public void run() {
                r.BasketExtension();
                telemetry.addData("Task", "BasketExtension");
                telemetry.update();
            }
        }, 1.5);

        // Task 3: At 2.5 seconds, run ClawOpen().
        PickupOne.addTimedAction(new Task() {
            @Override
            public void run() {
                r.ClawOpen();
                telemetry.addData("Task", "ClawOpen");
                telemetry.update();
            }
        }, 1.0);

        // Task 4: At 3.0 seconds, run BasketReturn().
        PickupOne.addTimedAction(new Task() {
            @Override
            public void run() {
                r.BasketReturn();
                telemetry.addData("Task", "BasketReturn");
                telemetry.update();
            }
        }, 1.0);





        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();
        r = new AutoRobot(hardwareMap, new Pose(7.000, 104.000));
        boxtube = r.boxtube;
        endEffector = r.endEffector;


        while(opModeInInit()){
            boxtube.updatePiv();
            boxtube.updateExt();
            r.InitPosition();
        }

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
            boxtube.changekP(0.005);
            switch (currentPathState) {

                case Preload:
                    follower.followPath(PreloadScore);
                    setPathState(PathStates.PickupOne);
                    PickupOne.Reset();
                    break;

                case PickupOne:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        PickupOne.update();
                    }
                    break;

                case ScoreOne:
                    if (!follower.isBusy()) {
                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
                            r.SampleHoverExt(12000, 0.52, 0.48);
                            r.ClawOpen();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 2.7) {
                            r.SampleGrab();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3.3) {
                            r.ClawClose();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 4){
                            r.LoiterSample();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() > 4){
                            follower.followPath(Score2);
                            setPathState(PathStates.PickupTwo);
                        }
                        else{r.Loiter();}
                    }
                    break;

                case PickupTwo:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if( actionTimer.getElapsedTimeSeconds() < 1.5) {
                            r.PivotBack();
                        }
                        else if( actionTimer.getElapsedTimeSeconds() < 2.5)
                        {
                            r.BasketExtension();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3){
                            r.ClawOpen();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3.5) {
                            r.BasketReturn();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() > 3.5){
                            r.Loiter();
                            follower.followPath(Pickup2);
                            setPathState(PathStates.ScoreTwo);
                        }

                    }
                    break;

                case ScoreTwo:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
                            r.SampleHoverExt(12000, 0.53, 0.48);
                            r.ClawOpen();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 2.7) {
                            r.SampleGrab();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3.3) {
                            r.ClawClose();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 4){
                            r.LoiterSample();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() > 4){
                            follower.followPath(Score2);
                            setPathState(PathStates.PickupThree);
                        }
                        else{r.Loiter();}
                    }
                    break;

                case PickupThree:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if( actionTimer.getElapsedTimeSeconds() < 1.5) {
                            r.PivotBack();
                        }
                        else if( actionTimer.getElapsedTimeSeconds() < 2.5)
                        {
                            r.BasketExtension();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3){
                            r.ClawOpen();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() < 3.5) {
                            r.BasketReturn();
                        }
                        else if(actionTimer.getElapsedTimeSeconds() > 3.5){
                            r.Loiter();
                            follower.followPath(Pickup3);
                            setPathState(PathStates.ScoreThree);
                        }

                    }
                    break;

                case ScoreThree:
                    if (!follower.isBusy()) {
                        follower.followPath(Score2);
                        setPathState(PathStates.Park);
                    }
                    break;

                case Park:
                    if (!follower.isBusy()) {
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
                                new Point(7.000, 104.000, Point.CARTESIAN),
                                new Point(21.676, 104.177, Point.CARTESIAN),
                                new Point(14.000, 129.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.500, 128.200, Point.CARTESIAN),
                                new Point(23.750, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();
//
        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(23.750, 131.000, Point.CARTESIAN),
                                new Point(15.500, 128.200, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.500, 128.200, Point.CARTESIAN),
                                new Point(23.750, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(23.750, 131.000, Point.CARTESIAN),
                                new Point(15.500, 128.200, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.500, 128.200, Point.CARTESIAN),
                                new Point(23.750, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();
//
        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(23.750, 131.000, Point.CARTESIAN),
                                new Point(15.500, 128.200, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.500, 128.200, Point.CARTESIAN),
                                new Point(31.253, 111.907, Point.CARTESIAN),
                                new Point(63.179, 118.460, Point.CARTESIAN),
                                new Point(59.750, 92.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(0.25)
                .build();
    }
}


