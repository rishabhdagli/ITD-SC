package config.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleRobot {
    Boxtube boxtube;
    EndEffector endEffector;
    double minExtension = 2000;
    double midExtension = 10000;
    double fullExtension = 20000;
    double basketExtension = 30500;
    double specScoreExtension = 20000;

    final int pivotBackPos = 1175;

    final int pivotHorizontal = 0;

    final int pivotSpecSpos = 700;

    Drivetrain drivetrain;

    public Telemetry t;

    Gamepad gamepad;


    public TeleRobot(HardwareMap hardwareMap, Telemetry t, Gamepad g2){
        this.t = t;

        boxtube = new Boxtube(hardwareMap,t);
        drivetrain = new Drivetrain(hardwareMap, t);

        gamepad = g2;
    }

    public void ClawOpen() {
        endEffector.claw(0.65);
    }

    public void ClawClose() {
        endEffector.claw(1);
    }

    public void Loiter() {
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(minExtension);
        endEffector.hand(0.48);
        endEffector.setEndEffector(80, -110);
        endEffector.turret(0.47);
        ClawOpen();
    }

    public void SampleHover(){
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(minExtension);
        endEffector.setEndEffector(10, -115);
        endEffector.turret(0.47);
        ClawOpen();

        //Add stuff to move extension + hand stuff using new .copy() method
    }

    public void SampleGrab() {
        boxtube.PivotMove(pivotHorizontal);
        endEffector.turret(0.47);
        endEffector.setEndEffector(-25,-75);
        ClawOpen();
    }

    public void LoiterSample() {
        boxtube.PivotMove(pivotHorizontal); // pivot should be horizontal
        boxtube.ExtensionMove(minExtension); //Make sure this is always in
        endEffector.setEndEffector(80, -110);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();

    }

    public void PivotBack(){
        boxtube.PivotMove(pivotBackPos);
        boxtube.ExtensionMove(minExtension); //Make sure this is always in when rotating
        endEffector.setEndEffector(-20,60); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void BasketExtension(){ // Ready to score
        boxtube.PivotMove(pivotBackPos);
        boxtube.ExtensionMove(basketExtension);
        endEffector.setEndEffector(-10,20); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void BasketReturn(){
        endEffector.setEndEffector(-40,0);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        boxtube.ExtensionMove(minExtension);
        boxtube.PivotMove(pivotBackPos - 200);
        ClawOpen();
    }

    public void obsZoneRelease(){
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(fullExtension);
        endEffector.setEndEffector(80,-110);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void Score() {
        ClawOpen();
    }


    //SPEC POSITIONS


    public void SpecimenWall() {
        boxtube.ExtensionMove(10000); // half extension
        boxtube.PivotMove(pivotHorizontal); // half pivot
        endEffector.hand(0.48);
        endEffector.setEndEffector(80, -80);
        endEffector.turret(0.47);
        ClawOpen();

    }

    public void SpecimenWallGrab() {
        boxtube.ExtensionMove(10000);
        boxtube.PivotMove(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.setEndEffector(80,-80);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void SpecimenWallUp(){
        boxtube.ExtensionMove(10000);
        boxtube.PivotMove(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        endEffector.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.ExtensionMove(minExtension);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        boxtube.PivotMove(pivotSpecSpos);
        endEffector.setEndEffector(100,-45);
    }

    public void SpecimenLatch() {
        boxtube.ExtensionMove(specScoreExtension);
    }
}
