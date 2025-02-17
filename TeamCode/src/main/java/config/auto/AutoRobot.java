package config.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.Subsystem.Boxtube;
import config.Subsystem.EndEffector;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class AutoRobot {

    public DcMotorEx LF,LR,RF,RR;
    public Boxtube boxtube;
    public EndEffector endEffector;
    double specPreScore = 0;
    double specScore = 15250;

    double specGrabExt = 10000;
    double sampleScore = 30500;
    double samplePickupExt = 2000;
    double pivotHorizontal = 0;
    double pivotSpecScore = 900;
    double pivotPreLoad = 550;
    double pivotBackPos = 1200;

    public Telemetry t;
    public Follower follower;

    Gamepad gamepad;

    public AutoRobot(HardwareMap h, Telemetry t, Pose startPose) {
        this.t = t;

//        LF = h.get(DcMotorEx.class,"leftFront");
//        LR = h.get(DcMotorEx.class,"leftBack");
//        RF = h.get(DcMotorEx.class,"rightFront");
//        RR = h.get(DcMotorEx.class,"rightBack");

        Constants.setConstants(FConstants.class, LConstants.class);

        boxtube = new Boxtube(h, t);
        endEffector = new EndEffector(h, t);
        follower = new Follower(h);

        follower.setStartingPose(startPose);
    }
    public void ChangePivotKP(double kp){
        boxtube.changekP(kp);
    }


    public void ClawOpen() {
        endEffector.claw(0.55);
    }

    public void ClawClose() {
        endEffector.claw(1);
    }

    public void InitPosition(){
        endEffector.setEndEffector(100, -110);
        endEffector.turret(0.2);
        endEffector.hand(47);
        boxtube.setPivot(200);
        boxtube.setExt(0);
        ClawClose();
    }

    public void Loiter(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(80, -110);
        endEffector.turret(0.47);
        boxtube.setPivot(0);
        boxtube.setExt(0);
        ClawClose();
    }

    public void LoiterIn() {
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        endEffector.setEndEffector(110, -20);
        boxtube.setPivot(0);
        boxtube.setExt(0);
    }

    public void SpecimenPreLoad() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-15);
        endEffector.turret(0.47);
        boxtube.setPivot(pivotPreLoad);
        boxtube.setExt(0);
        ClawClose();
    }

    public void PreloadSpecExt(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-15);
        endEffector.turret(0.47);
        boxtube.setPivot(pivotPreLoad);
        ClawClose();
        boxtube.setExt(13750);
    };

    public void SpecimenPreLoadScore(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-15);
        endEffector.turret(0.47);
        boxtube.setPivot(50);
        ClawClose();
    }

    public void SpecimenWall(){
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.setEndEffector(60,-45);
        endEffector.turret(0.47);
        ClawOpen();
    }

    public void SpecimenWallGrab() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(60,-45);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.setExt(specPreScore);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        endEffector.setEndEffector(45,70);
        ClawClose();
    }

    public void SpecimenLatch() {
        boxtube.setExt(specScore);
    }

    public void SpecimenExtDown()
    {
        boxtube.setExt(0);
    }

    public void PivotUpSpec(){
        boxtube.setPivot(pivotSpecScore);
    }


    //SAMPLE POSITIONS


    public void SampleHover(){
        boxtube.setPivot(pivotHorizontal);
        boxtube.setExt(samplePickupExt);
        endEffector.setEndEffector(30, -120);
        endEffector.turret(0.5);
        ClawOpen();
    }

    public void SampleGrab() {
        boxtube.setPivot(pivotHorizontal);
        endEffector.setEndEffector(-15,-55);
        ClawOpen();
    }

    public void SampleHoverExt(double ext, double turret, double hand){
        boxtube.setExt(ext);
        boxtube.setPivot(pivotHorizontal);
        endEffector.setEndEffector(30, -120);
        endEffector.turret(turret);
        endEffector.hand(hand);
    }

    public void LoiterSample() {
        boxtube.setPivot(pivotHorizontal); // pivot should be horizontal
        boxtube.setExt(samplePickupExt); //Make sure this is always in
        endEffector.setEndEffector(80, -110);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();

    }

    public void PivotBack(){
        boxtube.setPivot(pivotBackPos);
        boxtube.setExt(samplePickupExt); //Make sure this is always in when rotating
        endEffector.setEndEffector(-20,60); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void BasketExtension(){ // Ready to score
        boxtube.setPivot(pivotBackPos);
        boxtube.setExt(sampleScore);
        endEffector.setEndEffector(0,65); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void AutonReZero(){
        boxtube.setExt(0);
        boxtube.setPivot(pivotHorizontal);
        endEffector.setEndEffector(30, -120);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
    }

    public void BasketReturn(){
        endEffector.setEndEffector(-40,0);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        boxtube.setExt(samplePickupExt);
        boxtube.setPivot(pivotBackPos - 500);
        ClawOpen();
    }
}
