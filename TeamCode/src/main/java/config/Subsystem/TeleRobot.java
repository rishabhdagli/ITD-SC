package config.Subsystem;

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
    double specScoreExtension = 15250;
    double currentExtension =  midExtension;
    boolean wasPressedL;
    boolean wasPressedR;
    boolean minExtendSubPressed;
    boolean midExtendSubPressed;
    boolean lowExtendSubPressed;
    boolean maxExtendSubPressed;

    final int pivotBackPos = 1200;

    final int pivotHorizontal = 0;

    final int pivotSpecSpos = 900;

    public Telemetry t;

    Gamepad gamepad;


    public TeleRobot(HardwareMap hardwareMap, Telemetry t, Gamepad g2){
        this.t = t;

        boxtube = new Boxtube(hardwareMap,t);
        endEffector = new EndEffector(hardwareMap, t);

        gamepad = g2;
    }

    public void ClawOpen() {
        endEffector.claw(0.55);
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
        boxtube.ExtensionMove(currentExtension);
        endEffector.setEndEffector(30, -120);
        endEffector.turret(0.47);
        ClawOpen();

        if(gamepad.dpad_down){
            minExtendSubPressed = true;
            currentExtension = minExtension;
        }
        if(gamepad.dpad_left){
            midExtendSubPressed = true;
            currentExtension = midExtension;
        }
        if(gamepad.dpad_up){
            maxExtendSubPressed = true;
            currentExtension = fullExtension;
        }
        if(!gamepad.dpad_down && minExtendSubPressed){
            minExtendSubPressed  = false;
        }
        if(!gamepad.dpad_left && lowExtendSubPressed){
            lowExtendSubPressed = false;
        }
        if(!gamepad.dpad_right && midExtendSubPressed){
            midExtendSubPressed = false;
        }
        if(!gamepad.dpad_down && maxExtendSubPressed){
            maxExtendSubPressed = false;
        }

        if (gamepad.left_bumper) {
            wasPressedL = true;
        }
        if (gamepad.right_bumper) {
            wasPressedR = true;
        }

        if(!gamepad.left_bumper && wasPressedL){
            endEffector.hand(endEffector.handPos() + 0.05);
            wasPressedL = false;
        }
        if(!gamepad.right_bumper && wasPressedR){
            endEffector.hand(endEffector.handPos() - 0.05);
            wasPressedR = false;
        }

        //Add stuff to move extension + hand stuff using new .copy() method
    }

    public void SampleGrab() {
        boxtube.PivotMove(pivotHorizontal);
        endEffector.turret(0.47);
        endEffector.setEndEffector(-15,-55);
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
    public void HangUp(){
        boxtube.PivotMove(1147);//tbd
        endEffector.setEndEffector(30,-120); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
        if(boxtube.getPivpos() < -900){
            boxtube.ExtensionMove(14000); //tbd

        }
    }
    public void HangDown(){
        boxtube.ExtensionMove(0); //tbd
        endEffector.setEndEffector(30,-120); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
        if(boxtube.getExtpos() > -10000){
            boxtube.PivotMove(0);//tbd

        }
    }

    public void BasketExtension(){ // Ready to score
        boxtube.PivotMove(pivotBackPos);
        boxtube.ExtensionMove(basketExtension);
        endEffector.setEndEffector(0,65); //Needs fixing
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void BasketReturn(){
        endEffector.setEndEffector(-40,0);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        boxtube.ExtensionMove(minExtension);
        boxtube.PivotMove(pivotBackPos - 500);
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
        boxtube.ExtensionMove(minExtension); // half extension
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.setEndEffector(60,-45);
        endEffector.turret(0.47);
        ClawOpen();

    }

    public void SpecimenWallGrab() {
        boxtube.ExtensionMove(minExtension);
        boxtube.PivotMove(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.setEndEffector(60,-45);
        endEffector.turret(0.47);
        ClawClose();
    }

    public void SpecimenWallUp(){
        boxtube.ExtensionMove(minExtension);
        boxtube.PivotMove(pivotHorizontal);
        endEffector.hand(0.48);
        endEffector.turret(0.47);
        endEffector.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.ExtensionMove(0);
        endEffector.turret(0.47);
        endEffector.hand(0.48);
        boxtube.PivotMove(pivotSpecSpos);
        endEffector.setEndEffector(45,50);
    }

    public void SpecimenLatch() {
        boxtube.ExtensionMove(specScoreExtension);
    }
}
