package config.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import config.teleop.Boxtube;
import config.teleop.EndEffector;

public class AutoRobot {
    public Boxtube boxtube;
    public EndEffector endEffector;
    double specPreScore = 2000;
    double specScore = 20000;

    public Telemetry t;
    public Follower follower;

    Gamepad gamepad;

    public AutoRobot(HardwareMap h, Telemetry t, Pose startPose) {
        this.t = t;

        boxtube = new Boxtube(h, t);
        follower = new Follower(h);

        follower.setStartingPose(startPose);
    }

    public void ClawOpen() {
        endEffector.claw(0.65);
    }

    public void ClawClose() {
        endEffector.claw(1);
    }

    public void Loiter(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(80, -110);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void SpecimenPreLoad() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(0,0);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void SpecimenWall(){
        endEffector.hand(0.48);
        endEffector.setEndEffector(80,-80);
        endEffector.turret(0.47);
        ClawOpen();
    }

    public void SpecimenWallGrab() {
        endEffector.hand(0.48);
        endEffector.setEndEffector(80,-80);
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
        boxtube.ExtensionMove(specPreScore);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        boxtube.PivotMove(700);
        endEffector.setEndEffector(100,-45);
    }

    public void SpecimenLatch() {
        boxtube.ExtensionMove(specScore);
    }

    public void SpecimenLatchOpen(){
        ClawOpen();
    }

    public boolean conditionalEndPivot(double target) {
        double currentPivot = boxtube.pivotoffset + (boxtube.Pivot.getCurrentPosition());
        currentPivot = -currentPivot;
        if (currentPivot >= target - 60 && currentPivot <= target + 60) {
            boxtube.Pivot.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    public boolean conditionalEndExtension(double target) {
        double currentExt = boxtube.BT1.getCurrentPosition();
        currentExt = -currentExt;
        if (currentExt >= target - 500 && currentExt <= target + 500) {
            boxtube.ExtensionPower(0);
            return true;
        } else {
            return false;
        }
    }
}
