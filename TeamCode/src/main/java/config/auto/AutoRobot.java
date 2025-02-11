package config.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.teleop.Boxtube;
import config.teleop.EndEffector;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class AutoRobot {

    public DcMotorEx LF,LR,RF,RR;
    public Boxtube boxtube;
    public EndEffector endEffector;
    double specPreScore = 2000;
    double specScore = 13500;

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

    public void ClawOpen() {
        endEffector.claw(0.55);
    }

    public void ClawClose() {
        endEffector.claw(0.9);
    }

    public void InitPosition(){
        endEffector.setEndEffector(100, -130);
        endEffector.turret(0.2);
        endEffector.hand(47);
        boxtube.setPivot(250);
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

    public void LoiterIn(){
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        endEffector.setEndEffector(110,-20);
        boxtube.setPivot(0);
        boxtube.setExt(0);
    }

    public void SpecimenPreLoad() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-55);
        endEffector.turret(0.47);
        boxtube.setPivot(460);
        boxtube.setExt(0);
        ClawClose();
    }

    public void PreloadSpecExt(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-55);
        endEffector.turret(0.47);
        boxtube.setPivot(460);
        ClawClose();
        boxtube.setExt(14500);
    };

    public void SpecimenPreLoadScore(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,-55);
        endEffector.turret(0.47);
        boxtube.setPivot(200);
        ClawClose();
    }

    public void SpecimenWall(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(90,-120);
        endEffector.turret(0.47);
        ClawOpen();
    }

    public void SpecimenWallGrab() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(90,-120);
        endEffector.turret(0.47);
        ClawClose();
    }


    public void SpecimenWallUp(){
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        endEffector.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.setExt(specPreScore);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        boxtube.setPivot(700);
        endEffector.setEndEffector(55,-20);
    }

    public void SpecimenLatch() {
        boxtube.setExt(specScore);
    }

    public void SpecimenLatchOpen(){
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        endEffector.setEndEffector(55,-20);
        ClawOpen();
    }

//    public boolean conditionalEndPivot(double target) {
//        double currentPivot = boxtube.pivotoffset + (boxtube.Pivot.getCurrentPosition());
//        currentPivot = -currentPivot;
//        if (currentPivot >= target - 60 && currentPivot <= target + 60) {
//            boxtube.Pivot.setPower(0);
//            return true;
//        } else {
//            return false;
//        }
//    }
//
//    public boolean conditionalEndExtension(double target) {
//        double currentExt = boxtube.BT1.getCurrentPosition();
//        currentExt = -currentExt;
//        if (currentExt >= target - 500 && currentExt <= target + 500) {
//            boxtube.ExtensionPower(0);
//            return true;
//        } else {
//            return false;
//        }
//    }
}
