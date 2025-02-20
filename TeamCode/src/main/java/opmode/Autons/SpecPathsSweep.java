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


import Subsystems.AutoRobot;
import Subsystems.Boxtube;
import Subsystems.EndEffector;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "5 Spec Sweep")
public class SpecPathsSweep extends LinearOpMode {
    AutoRobot r;

    Boxtube boxtube;

    EndEffector endEffector;

    public static Follower follower;
    private Timer pathTimer = new Timer();

    private Timer actionTimer = new Timer();
    public boolean pathDone = false;
    public boolean prevPathDone = false;



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

    enum PathStates {
        Preload, PrePush1, PrePush2, PrePush3, Push1, Push2, Push3, Pickup1, Pickup2, Pickup3, Pickup4, Score1, Score2, Score3, Score4, Park, End, WAIT1
    }

    PathStates currentPathState, lastPathState;

    private void setPathState(PathStates state) {
        currentPathState = state;
        pathTimer.resetTimer();
        pathDone = false;
    }

    private void resetActionTimer(){
        pathDone = true;
        if(pathDone && !prevPathDone){
            actionTimer.resetTimer();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPaths();
        r = new AutoRobot(hardwareMap, new Pose(7.000, 54.000));
        boxtube = r.boxtube;
        endEffector = r.endEffector;
        r.t.addData("init", true);
        r.t.update();

        while(opModeInInit()){
            boxtube.updatePiv();
            boxtube.updateExt();
            r.InitPosition();
        }

        waitForStart();

        currentPathState = PathStates.Preload;
        actionTimer.resetTimer();
        pathTimer.resetTimer();

        while(opModeIsActive()){
            boxtube.updatePiv();
            boxtube.updateExt();
            follower.update();
            telemetry.addData("State", currentPathState);
            telemetry.update();

            switch(currentPathState){

                case Preload:
                    follower.followPath(Preload);
//                    resetActionTimer();
//                    r.Loiter();
//                    r.SpecimenPreLoad();
                    setPathState(PathStates.PrePush1);
                    break;

                case PrePush1:
                    if (!follower.isBusy()) {
                        resetActionTimer();
//                        r.PreloadSpecExt();
                        if(actionTimer.getElapsedTimeSeconds() > 0.5){
//                            r.SpecimenPreLoadScore();
                            if(actionTimer.getElapsedTimeSeconds() > 1){
//                                r.LoiterIn();
                                follower.followPath(PrePush1);
                                setPathState(PathStates.Push1);
                            }
                        }
                    }
                    break;

                case Push1:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() >1){
                        follower.followPath(Push1);
                        setPathState(PathStates.PrePush2);}
                    }
                    break;

                case PrePush2:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds()>1){
                        follower.followPath(PrePush2);
                        setPathState(PathStates.Push2);}
                    }
                    break;

                case Push2:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds()>1){
                            follower.followPath(Push2);
                        setPathState(PathStates.PrePush3);}
                    }
                    break;

                case PrePush3:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds()>1){
                            follower.followPath(PrePush3);
                        setPathState(PathStates.Push3);}
                    }
                    break;

                case Push3:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds()>1){
                            follower.followPath(Push3);
                            r.SpecimenWall();
                            setPathState(PathStates.Pickup1);}
                    }
                    break;
                    
                case Pickup1:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds()>1){
                            follower.followPath(Pickup1);
                        setPathState(PathStates.Score1);}
                    }
                    break;

                case Score1:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds()>0.05) {
                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
//                                r.SpecimenPreScore();
                                if(actionTimer.getElapsedTimeSeconds() > 0.35){
                                    follower.followPath(Score1);
                                    setPathState(PathStates.Pickup2);
                                }
                            }
                        } else {
//                            r.SpecimenWall();
                        }
                    }
                    break;

                case Pickup2:
                    if(!follower.isBusy()){
//                        r.PivotUpSpec();
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 0.7){
//                            r.SpecimenLatch();
                                if(actionTimer.getElapsedTimeSeconds()>1.3) {
//                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 1.35){
//                                        r.SpecimenWall();
                                        follower.followPath(Pickup2);
                                        setPathState(PathStates.Score2);
                                }
                            }
                        }
                    }
                    break;

                case Score2:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds()>0.05) {
//                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
//                                r.SpecimenPreScore();
                                if(actionTimer.getElapsedTimeSeconds() > 1){
                                    follower.followPath(Score2);
                                    setPathState(PathStates.Pickup3);
                                    }
                            }
                        } else {
//                            r.SpecimenWall();
                        }
                    }
                    break;

                case Pickup3:
                    if(!follower.isBusy()) {
//                        r.PivotUpSpec();
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 0.7){
//                            r.SpecimenLatch();
                                if(actionTimer.getElapsedTimeSeconds()>1.3) {
//                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 1.35){
//                                        r.SpecimenWall();
                                        follower.followPath(Pickup3);
                                        setPathState(PathStates.Score3);
                                }
                            }
                        }
                    }
                    break;

                case Score3:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds()>0.05) {
//                            r.SpecimenWallGrab();
                            if (actionTimer.getElapsedTimeSeconds() > 0.1) {
//                                r.SpecimenPreScore();
                                if (actionTimer.getElapsedTimeSeconds() > 1) {
                                    follower.followPath(Score3);
                                    setPathState(PathStates.Pickup4);
                                }
                            } else {
//                                r.SpecimenWall();
                            }
                        }
                    }
                    break;

                case Pickup4:
                    if(!follower.isBusy()){
//                        r.PivotUpSpec();
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 0.7){
//                            r.SpecimenLatch();
                                if(actionTimer.getElapsedTimeSeconds()>1.3) {
//                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 1.35){
//                                        r.SpecimenWall();
                                        follower.followPath(Pickup4);
                                        setPathState(PathStates.Score4);
                                }
                            }
                        }
                    }
                    break;

                case Score4:
                    if(!follower.isBusy()){
//                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds()>0.05) {
//                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 0.1) {
//                                r.SpecimenPreScore();
                                if(actionTimer.getElapsedTimeSeconds() > 1){
                                    follower.followPath(Score4);
                                    setPathState(PathStates.Park);
                                }
                            }
                        } else {
//                            r.SpecimenWall();
                        }
                    }
                    break;

                case Park:
                    if(!follower.isBusy()){
//                        r.PivotUpSpec();
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 0.7){
//                            r.SpecimenLatch();
                                if(actionTimer.getElapsedTimeSeconds()>1.3) {
//                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 1.35){
//                                        r.SampleHover();
                                        follower.followPath(Park);
//                                        setPathState(PathStates.End);
                                    }
                            }
                        }
                    }
                    break;

                case End:
                default:
                    if(!follower.isBusy()){
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
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setZeroPowerAccelerationMultiplier(.75)
                    .build();

            PrePush1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(25.000, 61.000, Point.CARTESIAN),
                                    new Point(30.500, 36.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                    .build();

            Push1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(30.500, 36.000, Point.CARTESIAN),
                                    new Point(27.000, 33.500, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-120))
                    .build();

            PrePush2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(27.000, 33.500, Point.CARTESIAN),
                                    new Point(31.250, 29.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-30))
                    .build();

            Push2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(57.000, 16.000, Point.CARTESIAN),
                                    new Point(30.000, 13.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-120))
                    .build();

            PrePush3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(31.250, 29.000, Point.CARTESIAN),
                                    new Point(26.000, 26.000, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-45))
                    .build();

            Push3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(26.000, 26.000, Point.CARTESIAN),
                                    new Point(31.589, 20.331, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                    .build();

            Pickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(31.589, 20.331, Point.CARTESIAN),
                                    new Point(37.806, 38.982, Point.CARTESIAN),
                                    new Point(20.331, 16.803, Point.CARTESIAN),
                                    new Point(12.266, 23.860, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .setZeroPowerAccelerationMultiplier(0.75)
                    .build();

            Score1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(12.266, 23.860, Point.CARTESIAN),
                                    new Point(32.093, 70.572, Point.CARTESIAN),
                                    new Point(40.831, 64.859, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .setZeroPowerAccelerationMultiplier(1.75)
                    .build();

            Pickup2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(40.831, 64.859, Point.CARTESIAN),
                                    new Point(32.093, 70.572, Point.CARTESIAN),
                                    new Point(12.266, 23.860, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-180))
                    .setZeroPowerAccelerationMultiplier(0.75)
                    .build();
        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.266, 23.860, Point.CARTESIAN),
                                new Point(32.093, 70.572, Point.CARTESIAN),
                                new Point(40.831, 64.859, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(40.831, 64.859, Point.CARTESIAN),
                                new Point(32.093, 70.572, Point.CARTESIAN),
                                new Point(12.266, 23.860, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();
        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.266, 23.860, Point.CARTESIAN),
                                new Point(32.093, 70.572, Point.CARTESIAN),
                                new Point(40.831, 64.859, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(40.831, 64.859, Point.CARTESIAN),
                                new Point(32.093, 70.572, Point.CARTESIAN),
                                new Point(12.266, 23.860, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();
        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.266, 23.860, Point.CARTESIAN),
                                new Point(32.093, 70.572, Point.CARTESIAN),
                                new Point(40.831, 64.859, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .setZeroPowerAccelerationMultiplier(1.75)
                .build();


//            Score2 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Point(19.500, 30.000, Point.CARTESIAN),
//                                    new Point(12.903, 62.000, Point.CARTESIAN),
//                                    new Point(44.750, 73.000, Point.CARTESIAN)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setZeroPowerAccelerationMultiplier(1.75)
//                    .build();
//
//            Pickup3 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Point(44.750, 73.000, Point.CARTESIAN),
//                                    new Point(24.000, 62.000, Point.CARTESIAN),
//                                    new Point(55.000,35.000, Point.CARTESIAN),
//                                    new Point(19.500, 30.000, Point.CARTESIAN)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setZeroPowerAccelerationMultiplier(1.25)
//                    .build();
//
//            Score3 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Point(19.500, 30.000, Point.CARTESIAN),
//                                    new Point(12.903, 62.000, Point.CARTESIAN),
//                                    new Point(44.750, 77.000, Point.CARTESIAN)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setZeroPowerAccelerationMultiplier(1.75)
//                    .build();
//
//            Pickup4 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Point(44.750, 77.000, Point.CARTESIAN),
//                                    new Point(24.000, 62.000, Point.CARTESIAN),
//                                    new Point(55.000,35.000, Point.CARTESIAN),
//                                    new Point(19.500, 30.000, Point.CARTESIAN)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setZeroPowerAccelerationMultiplier(1.25)
//                    .build();
//
//            Score4 = follower.pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Point(19.500, 30.000, Point.CARTESIAN),
//                                    new Point(12.903, 62.000, Point.CARTESIAN),
//                                    new Point(44.750, 80.000, Point.CARTESIAN)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .setZeroPowerAccelerationMultiplier(1.75)
//                    .build();
            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(44.750, 80.000, Point.CARTESIAN),
                                    new Point(22.000, 30.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
}