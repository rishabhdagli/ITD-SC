package config.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot implements Subsystem {
    DiddyArm boxtube;
    public boolean minExtendSubPressed = false, maxExtendSubPressed = false, lowExtendSubPressed = false, midExtendSubPressed  = false,  wasPressedL, wasPressedR;
    double minExtensionandLowextention = 2000;
    double midExtension = 10000;
    double FarSample = 20000;
    double highBasket = 30500;
    double currentExtension = midExtension;

    double specScore = 20000;

    final int PivotOverCenterPosition = 1175;

    final int PivotSpecScore = 700;

    DragonWarrior drivetrain;

    Gamepad gamepad;

    @Override
    public void init(HardwareMap hardwareMap) {
        boxtube = new DiddyArm();
        drivetrain = new DragonWarrior();

        boxtube.init(hardwareMap);
        drivetrain.init(hardwareMap);

    }

    public void init(HardwareMap hardwareMap, Gamepad g2) {
        boxtube = new DiddyArm();
        drivetrain = new DragonWarrior();

        boxtube.init(hardwareMap);
        drivetrain.init(hardwareMap);

        gamepad = g2;

    }

    //MAKE ROBOT SLOWER WHEN PICKING UP FROM SUB


    public void ClawOpen() {
        boxtube.claw(0.65);
    }

    public void ClawClose() {
        boxtube.claw(1);
    }



    public void Loiter() {
       boxtube.PivotMove(0); // pivot should be horizontal
        boxtube.ExtensionMove(minExtensionandLowextention);
        boxtube.hand(0.48);
        boxtube.setEndEffector(80, -110);
        boxtube.turret(0.47);
        ClawOpen();
        // no claw bc its used twice before grabbing sample and then after grabbing sample
    }

    public void LoiterAuto(){
        boxtube.hand(0.48);
        boxtube.setEndEffector(80, -110);
        boxtube.turret(0.47);
        ClawClose();
    }
    public void zero(){
        boxtube.hand(0.48);
        boxtube.setEndEffector(0,0);
        boxtube.turret(0.47);
        ClawClose();
    }



    public void SampleHover() {
        boxtube.PivotMove(0); // pivot should be horizontal
//        boxtube.ExtensionMove();
        //change later to varibale distances
        boxtube.ExtensionMove(currentExtension);
        boxtube.setEndEffector(10, -115);
        boxtube.turret(0.47);
        ClawOpen();

//        if (gamepad.dpad_up) {
//            extendSubPressed = true;
//        }
//        if (gamepad.dpad_down) {
//            deExtendSubPressed = true;
//        }
//        if(!gamepad.dpad_up && extendSubPressed){
//            boxtube.ExtensionMove((boxtube.ExtensionReturn() + 10000));
//            extendSubPressed = false;
//        }
//        if(!gamepad.dpad_right && deExtendSubPressed){
//            boxtube.ExtensionMove((boxtube.ExtensionReturn() - 10000));
//            deExtendSubPressed = false;
//        }

        //if dpad.down is clicked, the extension will go to 5000
        //if dpad.left is clicked the extension will go to 10000
        //if dpad.up is clicked the extension will go to 20000

        if(gamepad.dpad_down){
            minExtendSubPressed = true;
            currentExtension = minExtensionandLowextention;
        }
        if(gamepad.dpad_left){
            midExtendSubPressed = true;
            currentExtension = midExtension;
        }
        if(gamepad.dpad_up){
            maxExtendSubPressed = true;
            currentExtension = FarSample;
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
            boxtube.hand(boxtube.handPos() + 0.05);
            wasPressedL = false;
        }
        if(!gamepad.right_bumper && wasPressedR){
            boxtube.hand(boxtube.handPos() - 0.05);
            wasPressedR = false;
        }
//   wrist angle -90
    }

//    public void AfterGrab(){
//        // Same as Sample hover, but with claw closed, so that sample is held
//        boxtube.setEndEffector(80, -120);
//        boxtube.turret(0.47);
//        boxtube.PivotMove(0); // pivot should be horizontal
//        boxtube.ExtensionMove(0);
//        ClawClose();
//    }

    public void SampleGrab() {
        //getting to the closed position
        boxtube.PivotMove(0
        ); // pivot should be horizontal
        boxtube.ExtensionMove(currentExtension);
        boxtube.turret(0.47);
        boxtube.setEndEffector(-25,-75);
        ClawOpen();
    }

    public void LoiterSample() {
        boxtube.PivotMove(0); // pivot should be horizontal
        boxtube.ExtensionMove(minExtensionandLowextention); //Make sure this is always in
//        boxtube.setArmAngle(70);
//        boxtube.setWristAngle(-100);
        boxtube.setEndEffector(80,-110);
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        ClawClose();

    }
    public void PivotBack(){
        boxtube.PivotMove(PivotOverCenterPosition); // pivot should be horizontal
        boxtube.ExtensionMove(minExtensionandLowextention); //Make sure this is always in
//        boxtube.setArmAngle(70);
//        boxtube.setWristAngle(-100);
        boxtube.setEndEffector(80,-120);
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        ClawClose();
    }




    public void MovingPivotforHighBasket(){
        boxtube.PivotMove(PivotOverCenterPosition);
        boxtube.ExtensionMove(minExtensionandLowextention); //Make sure this is always in when rotating
        boxtube.setEndEffector(-20,60); //Needs fixing
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        ClawClose();
    }

    public void MovingExtentionforHighBasket(){ // Ready to score
        boxtube.PivotMove(PivotOverCenterPosition);
        boxtube.ExtensionMove(highBasket); //Make sure this is always in when rotating
        boxtube.setEndEffector(-10,20); //Needs fixing
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        ClawClose();
    }

    public void ObszoneScoreing(){
        boxtube.PivotMove(0);
        boxtube.ExtensionMove(FarSample);
        boxtube.setEndEffector(80,-110);
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        ClawClose();
    }



    public void BringExtentionDownAfterButton() {
        boxtube.ExtensionMove(minExtensionandLowextention);
//

    }
    public void Score() {
        ClawOpen();
    }
    public void BasketPos2(){
     //so it doesn't hang on high basket
        boxtube.setEndEffector(-40,0);
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        boxtube.ExtensionMove(minExtensionandLowextention);
        boxtube.PivotMove(PivotOverCenterPosition - 200);
        ClawOpen();
    }




    public void ObsZoneRelease() {
        // Extend out towards Obs zone
//        boxtube.wrist();
//        boxtube.hand();
//        boxtube.arm();
//        boxtube.turret();
    }

    public void ObsZoneReleaseScore() {
        // Make all of the positions come watever is best for sample release, and then have a smaple release state in state machine
//        ClawOpen();
    }


    public void SpecimenWall() {
        boxtube.ExtensionMove(10000); // half extension
        boxtube.PivotMove(0); // half pivot
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawOpen();

    }

    public void SpecimenWallAuto(){
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawOpen();
    }

    public void SpecimenWallGrab() {
        boxtube.ExtensionMove(10000);
        boxtube.PivotMove(0);
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawClose();
    }

    public void SpecimenWallGrabAuto() {
        boxtube.hand(0.48);
        boxtube.setEndEffector(80,-80);
        boxtube.turret(0.47);
        ClawClose();
    }

    //add wait time 1 second

    public void SpecimenWallUp(){
        boxtube.ExtensionMove(10000);
        boxtube.PivotMove(0);
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        boxtube.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenWallUpAuto(){
        boxtube.hand(0.48);
        boxtube.turret(0.47);
        boxtube.setEndEffector(100, -40);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.ExtensionMove(minExtensionandLowextention);
        boxtube.turret(0.47);
        boxtube.hand(0.48);
        boxtube.PivotMove(700);
        boxtube.setEndEffector(100,-45);
    }

    public void SpecimenLatch() {
        boxtube.ExtensionMove(specScore);
    }
    public void SpecimenLatch2(){
        ClawOpen();
    }
    public void PushingSamplePos(){
        boxtube.turret(0.47);
        boxtube.hand(0.48);
        boxtube.PivotMove(700);
        boxtube.setEndEffector(125,-30);
    }

}
