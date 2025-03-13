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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import Subsystems.Boxtube;
import Subsystems.EndEffector;
import Subsystems.Robot;
import opmode.Vision.CrushSampleAnglePipelineTurretTrial;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "4 Sample Auto Align")
public class SampleAutoReworked extends LinearOpMode {


    public static Follower follower;
    public Robot r;
    public VisionPortal VP;
    public CrushSampleAnglePipelineTurretTrial pipeline;
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
        Preload, PickupOne, ScoreOne, PickupTwo, ScoreTwo, PickupThree, ScoreThree,ActualScore3, Park, End
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
        r = new Robot(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7.000,104.000,Math.toRadians(-90)));
        buildPaths();
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
                    if(!follower.isBusy()) {
                        r.PivotBack();
                        follower.followPath(PreloadScore);
                        setPathState(PathStates.PickupOne);
                    }
                    break;

                case PickupOne: //this is scoring the preload
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if ( actionTimer.getElapsedTimeSeconds() < 1) {
                            r.BasketExtension();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 1.25) {
                            r.BasketScore();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() <2)
                        {
                            r.BasketReturn();
                        } else if (actionTimer.getElapsedTimeSeconds() < 2.75) {
                            r.PivotDown();
                            r.AutoSampleHover();
                        }
                        if (actionTimer.getElapsedTimeSeconds() >2.75) {
                            follower.followPath(Pickup1);
                            setPathState(PathStates.ScoreOne);
                        }

                    }
                    break;
                case ScoreOne: //picking up the first one
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (actionTimer.getElapsedTimeSeconds() < 1)
                        {
                            r.AutoSampleHover();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 2.5)
                        {
                            r.AutoAlign();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 3.0)
                        {
                            r.SampleGrab();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 3.25)
                        {
                            r.ClawClose();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 3.25){
                            r.LoiterSample();
                            follower.followPath(Score1);
                            setPathState(PathStates.PickupTwo);}
                    }
                    break;

                case PickupTwo: //score the first one
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if ( actionTimer.getElapsedTimeSeconds() < 1) {
                            r.PivotBack();
                        }
                        else if ( actionTimer.getElapsedTimeSeconds() < 2) {
                            r.BasketExtension();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 2.25) {
                            r.BasketScore();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() <2.75)
                        {
                            r.BasketReturn();
                        } else if (actionTimer.getElapsedTimeSeconds() < 3.25) {
                            r.PivotDown();
                            r.AutoSampleHover();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                            follower.followPath(Pickup2);
                            setPathState(PathStates.ScoreTwo);
                        }
                    }
                    break;

                case ScoreTwo: //pick up the second one
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if (actionTimer.getElapsedTimeSeconds() < 1)
                        {
                            r.AutoSampleHover();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 2.5)
                        {
                            r.AutoAlign();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 3.0)
                        {
                            r.SampleGrab();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 3.25)
                        {
                            r.ClawClose();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 3.25){
                            r.LoiterSample();
                            follower.followPath(Score2);
                            setPathState(PathStates.PickupThree);
                        }
                    }
                    break;

                case PickupThree: //score the third one
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (actionTimer.getElapsedTimeSeconds() < 1) {
                            r.PivotBack();
                        }
                        else if ( actionTimer.getElapsedTimeSeconds() < 2) {
                            r.BasketExtension();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 2.25) {
                            r.BasketScore();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() <2.75)
                        {
                            r.BasketReturn();
                        } else if (actionTimer.getElapsedTimeSeconds() < 3.25) {
                            r.PivotDown();
                            r.AutoSampleHoverForLastThing();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                            follower.followPath(Pickup3);
                            setPathState(PathStates.ScoreThree);
//                            r.PivotDown();
                        }
                    }
                    break;

                case ScoreThree:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (actionTimer.getElapsedTimeSeconds() < 0.5)
                        {
                            r.AutoSampleHoverForLastThing();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 1)
                        {
                            r.SampleGrab();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 1.25)
                        {
                            r.ClawClose();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 1.65){
                            r.LoiterSample();
                            follower.followPath(Score3);
                            setPathState(PathStates.ActualScore3);}
                    }
                    break;
                case ActualScore3:
                    if (!follower.isBusy()) {
                        resetActionTimer();
                        if (actionTimer.getElapsedTimeSeconds() < 1) {
                            r.PivotBack();
                        }
                        else if ( actionTimer.getElapsedTimeSeconds() < 2) {
                            r.BasketExtension();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() < 2.25) {
                            r.BasketScore();
                        }
                        else if (actionTimer.getElapsedTimeSeconds() <2.75)
                        {
                            r.BasketReturn();
                        } else if (actionTimer.getElapsedTimeSeconds() < 3.25) {
                            r.PivotDown();
                        }
                        if (actionTimer.getElapsedTimeSeconds() > 3.25) {
                            follower.followPath(Park);
                            setPathState(PathStates.Park);
                        }
                    }

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
                                new Point(7.000, 104.000, Point.CARTESIAN),
                                new Point(28.000, 119.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(27.000, 120.7500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(28.500, 120.000, Point.CARTESIAN),
                                new Point(14.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(14.000, 130.000, Point.CARTESIAN),
                                new Point(27.000, 131.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(27.500, 131.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(44.500, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();
//
        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(28.500, 137.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(0.75)
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
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();
    }
}


