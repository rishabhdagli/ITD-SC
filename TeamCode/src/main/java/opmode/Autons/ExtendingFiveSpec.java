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

@Autonomous(name = "Extending 5 Specimen")
public class ExtendingFiveSpec extends LinearOpMode {
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
        r = new AutoRobot(hardwareMap, telemetry, new Pose(7.000, 54.000));
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

                    resetActionTimer();

                    r.Loiter();
                    r.SpecimenPreLoad();
                    setPathState(PathStates.PrePush1);
                    break;

                case PrePush1:
                    if (!follower.isBusy()) {

                        resetActionTimer();

                        r.PreloadSpecExt();

                        if(actionTimer.getElapsedTimeSeconds() > 0.5){
                            r.SpecimenPreLoadScore();
                            if(actionTimer.getElapsedTimeSeconds() > 1.25){
                                r.LoiterIn();
                                follower.followPath(PrePush1);
                                setPathState(PathStates.Push1);
                            }
                        }
                    }
                    break;

                case Push1:
                    if(!follower.isBusy()){
                        follower.followPath(Push1);
                        setPathState(PathStates.PrePush2);
                    }
                    break;

                case PrePush2:
                    if(!follower.isBusy()){
                        follower.followPath(PrePush2);
                        setPathState(PathStates.Push2);
                    }
                    break;

                case Push2:
                    if(!follower.isBusy()){
                        follower.followPath(Push2);
                        setPathState(PathStates.PrePush3);
                    }
                    break;

                case PrePush3:
                    if(!follower.isBusy()){
                        follower.followPath(PrePush3);
                        setPathState(PathStates.Push3);
                    }
                    break;

                case Push3:
                    if(!follower.isBusy()){
                        follower.followPath(Push3);
                        r.SpecimenWall();
                        setPathState(PathStates.Pickup1);
                    }
                    break;

                case Pickup1:
                    if(!follower.isBusy()){
                        follower.followPath(Pickup1);
                        setPathState(PathStates.Score1);
                    }
                    break;

                case Score1:
                    if(!follower.isBusy()){

                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds()>0.5) {
                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 1) {
                                r.SpecimenWallUp();
                                follower.followPath(Score1);
                                r.SpecimenPreScore();
                                setPathState(PathStates.Pickup2);
                            }
                        } else {
                            r.SpecimenWall();
                        }
                    }
                    break;

                case Pickup2:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 0.75){
                            r.SpecimenLatch();
                            if(actionTimer.getElapsedTimeSeconds() > 1.25){
                                r.SpecimenLatchOpen();
                                if(actionTimer.getElapsedTimeSeconds()>2) {
                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 2.5){
                                        r.SpecimenWall();
                                        follower.followPath(Pickup2);
                                        setPathState(PathStates.Score2);
                                    }

                                }
                            }
                        }
                    }
                    break;

                case Score2:
                    if(!follower.isBusy()){
                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds()>0.5) {
                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 1) {
                                r.SpecimenWallUp();
                                follower.followPath(Score2);
                                r.SpecimenPreScore();
                                setPathState(PathStates.Pickup3);
                            }
                        } else {
                            r.SpecimenWall();
                        }
                    }
                    break;

                case Pickup3:
                    if(!follower.isBusy()) {
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 1.5){
                            r.SpecimenLatch();
                            if(actionTimer.getElapsedTimeSeconds() > 2.5){
                                r.SpecimenLatchOpen();
                                if(actionTimer.getElapsedTimeSeconds()>3.5) {
                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 4){
                                        r.SpecimenWall();
                                        follower.followPath(Pickup3);
                                        setPathState(PathStates.Score3);
                                    }

                                }
                            }
                        }
                    }
                    break;

                case Score3:
                    if(!follower.isBusy()){
                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds()>0.5) {
                            r.SpecimenWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 1) {
                                r.SpecimenWallUp();
                                follower.followPath(Score3);
                                r.SpecimenPreScore();
                                setPathState(PathStates.Pickup4);
                            }
                        } else {
                            r.SpecimenWall();
                        }
                    }
                    break;

                case Pickup4:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 1.5){
                            r.SpecimenLatch();
                            if(actionTimer.getElapsedTimeSeconds() > 2.5){
                                r.SpecimenLatchOpen();
                                if(actionTimer.getElapsedTimeSeconds()>3.5) {
                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 4){
                                        r.SpecimenExtendedWall();
                                        follower.followPath(Pickup4);
                                        setPathState(PathStates.Score4);
                                    }

                                }
                            }
                        }
                    }
                    break;

                case Score4:
                    if(!follower.isBusy()){
                        resetActionTimer();

                        if(actionTimer.getElapsedTimeSeconds()>1) {
                            r.SpecimenExtendedWallGrab();
                            if(actionTimer.getElapsedTimeSeconds() > 2) {
                                r.SpecimenExtendedWallUp();
                                if(actionTimer.getElapsedTimeSeconds() > 2.25){
                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 2.5){
                                        follower.followPath(Score4);
                                        r.SpecimenPreScore();
                                        setPathState(PathStates.Park);
                                    }
                                }
                            }
                        } else {
                            r.SpecimenExtendedWall();
                        }
                    }
                    break;

                case Park:
                    if(!follower.isBusy()){
                        resetActionTimer();
                        if(actionTimer.getElapsedTimeSeconds() > 1.5){
                            r.SpecimenLatch();
                            if(actionTimer.getElapsedTimeSeconds() > 2.5){
                                r.SpecimenLatchOpen();
                                if(actionTimer.getElapsedTimeSeconds()>3.5) {
                                    r.SpecimenExtDown();
                                    if(actionTimer.getElapsedTimeSeconds() > 4){
                                        r.LoiterIn();
                                        follower.followPath(Park);
                                        setPathState(PathStates.End);
                                    }
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
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(.75)
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
                                new Point(22.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(22.000, 25.000, Point.CARTESIAN),
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
                                new Point(22.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PrePush3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(22.000, 13.000, Point.CARTESIAN),
                                new Point(32.000, 11.871, Point.CARTESIAN),
                                new Point(59.100, 18.050, Point.CARTESIAN),
                                new Point(57, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Push3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57, 8.000, Point.CARTESIAN),
                                new Point(19.750, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(19.750, 8.000, Point.CARTESIAN),
                                new Point(21.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(21.000, 8.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(44.000, 69.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.1)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(44.000, 69.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000,35.000, Point.CARTESIAN),
                                new Point(21.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(21.000, 30.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(44.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.5)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(44.000, 73.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000,35.000, Point.CARTESIAN),
                                new Point(21.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Score3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(21.000, 30.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(44.000, 77.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.1)
                .build();

        Pickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(44.000, 77.000, Point.CARTESIAN),
                                new Point(24.000, 62.000, Point.CARTESIAN),
                                new Point(55.000,35.000, Point.CARTESIAN),
                                new Point(21.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Score4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(21.000, 30.000, Point.CARTESIAN),
                                new Point(12.903, 62.000, Point.CARTESIAN),
                                new Point(44.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.1)
                .build();
        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(44.000, 80.000, Point.CARTESIAN),
                                new Point(22.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}