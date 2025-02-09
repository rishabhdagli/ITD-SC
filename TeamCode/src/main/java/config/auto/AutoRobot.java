package config.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoRobot {
    public Boxtube boxtube;
    double prescore = 2000;
    double specScore = 20000;

    public Telemetry t;
    public Follower follower;

    public AutoRobot(HardwareMap h, Telemetry t, Pose startPose) {
        this.t = t;

        boxtube = new Boxtube(h, t);
        follower = new Follower(h);

        follower.setStartingPose(startPose);
    }

    public void ClawOpen() {
        boxtube.claw(0.65);
    }

    public void ClawClose() {
        boxtube.claw(1);
    }

    public void Loiter(){
        boxtube.hand(0.48);
        boxtube.setEndEffector(80, -110);
        boxtube.turret(0.47);
        ClawClose();
    }

    public void SpecimenWall(){
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawOpen();
    }

    public void SpecimenWallGrab() {
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawClose();
    }


    public void SpecimenWallUp(){
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        boxtube.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.ExtensionMove(prescore);
        boxtube.turret(0.47);
        boxtube.hand(0.48);
        boxtube.PivotMove(700);
        boxtube.setEndEffector(100,-45);
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

    public void zero() {
        boxtube.hand(0.48);
        boxtube.setEndEffector(0,0);
        boxtube.turret(0.47);
        ClawClose();
    }
}
