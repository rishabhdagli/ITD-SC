package Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import opmode.MainTele;
import opmode.Vision.AutoAlignTuner;
import opmode.Vision.CrushSampleAnglePipelineTurretTrial;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Robot {

    public Boxtube boxtube;
    public EndEffector endEffector;
    //Pedro
    public Follower follower;
    //COMMON
    ElapsedTime timer;
    public Drivetrain drive;


    //PIVOT VARIABLES

    //over shoort 1120 to ensure locking
    double pivotBackPos = 1280, pivotHorizontal = 0, pivotSpecSpos = 900, pivotPreLoad = 550;


    //EXTENSION VARIABLES
    double minExtension = 000, midExtension = 12500, fullExtension = 25000, basketExtension = 56000, specScoreExtension = 15250, currentExtension = minExtension, specimenWallExtension = 15000;


    //For Endeffecotor
    double hand = 0.17; boolean horizontal = true;

    //BOOLEANS FOR BUTTONS
    boolean wasPressedL, wasPressedR, minExtendSubPressed, midExtendSubPressed,
            lowExtendSubPressed, maxExtendSubPressed,wasPressedTriangle = false;
    //Gamepads
    Gamepad gamepadOperator, gamepadDriver;
    //VISION
    public CrushSampleAnglePipelineTurretTrial pipeline;
    public VisionPortal VP;


    //AutoAlign Varibles

    enum AutoAlignstates {
        TURRET_ALIGN, HAND_ALIGN, PICK_UP,NOT_DETECTED
    }
    private AutoAlignstates Camerastate = AutoAlignstates.TURRET_ALIGN;
    private AutoAlignTuner.BoxtubeParam bp = new AutoAlignTuner.BoxtubeParam();
    private AutoAlignTuner.CameraParams cp = new AutoAlignTuner.CameraParams();
    private AutoAlignTuner.TurretParams tp = new AutoAlignTuner.TurretParams();
    private AutoAlignTuner.HandParams hp = new AutoAlignTuner.HandParams();

    public double ServoRegulizer(double x) {
        return (x > 1) ? (((int)(x * 100)) % 100) / 100.0 : x;
    }

    public boolean AutoAlign = false;
















    // Constructors for auto-robot and tele-robot


    public Robot(HardwareMap hardwareMap, Gamepad g1, Gamepad g2) {

        drive = new Drivetrain(hardwareMap);
        boxtube = new Boxtube(hardwareMap, 1); //init without reseting 0
        endEffector = new EndEffector(hardwareMap);

        gamepadOperator = g2;
        gamepadDriver = g1;
    }

    public Robot(HardwareMap h) {
        boxtube = new Boxtube(h);
        endEffector = new EndEffector(h);
        follower = new Follower(h);

        currentExtension = boxtube.getExtpos();

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        pipeline = new CrushSampleAnglePipelineTurretTrial();
       VP = VisionPortal.easyCreateWithDefaults(h.get(WebcamName.class, "Webcam 1"), pipeline);

    }






















    //TELE - Op drive control
    public void TeleControl(double yMultiplier, double xMultiplier, double rxMultiplier) {
        drive.TeleopControl(yMultiplier * gamepadDriver.left_stick_y, xMultiplier * gamepadDriver.left_stick_x, rxMultiplier * gamepadDriver.right_stick_x);
    }

    public void UPDATE(){
        drive.update();
        boxtube.update();
    }



















    //AUTON SPECIFIC METHOD HERE

    public void AutoSampleHover() {
        boxtube.setPivot(pivotHorizontal);
        endEffector.arm(0.44);
        endEffector.wrist(0.11);
        endEffector.turret(0.55);
        ClawOpen();
    }
    public void AutoSampleHoverForLastThing() {
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.485);
        endEffector.arm(0.46);
        endEffector.wrist(0.14);
        endEffector.turret(0.55);
        ClawOpen();
    }

    public void InitPosition() {
        boxtube.setExt(minExtension);
        boxtube.setPivot(420);
        endEffector.hand(0.17);
        endEffector.wrist(0.63);
        endEffector.arm(0.15);
        endEffector.claw(0.4);
        endEffector.turret(0.55);
        ClawClose();
    }


    public void SpecimenWallForSamplePush() {
        boxtube.setExt(minExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.17);
        endEffector.wrist(0.45);
        endEffector.arm(0.4);
        endEffector.turret(0.55);
        ClawOpen();

    }


    public void PivotDown(){
        boxtube.setPivot(0);
    }

    public void AutoAlign(){
        switch (Camerastate)
        {
            case TURRET_ALIGN:
                bp.ErrorY =  pipeline.getMiddleLineY() - cp.middleLine;
                //boxtube.ExtensionPower(bp.KpExtention*bp.ErrorY);
                tp.error  =  cp.verticalCenterFrame - pipeline.getMiddleLineX();
                tp.ServoGain =  tp.AGain*(tp.error*tp.error) + tp.BGain*(tp.error) + tp.CGain;


                if(Math.abs(tp.error) >= 321){
                    //do nothing
                }
                //moving turret
                else if(cp.thresholdPixelError < Math.abs(tp.error)) {
                    if (tp.error > 0) {
                        endEffector.turret(endEffector.Turret.getPosition() + tp.ServoGain);
                    } else if (tp.error < 0) {
                        endEffector.turret(endEffector.Turret.getPosition() - tp.ServoGain);
                    }
                }

                //if in position then move Hand
                else if (cp.thresholdPixelError >=  Math.abs(tp.error)){
                    Camerastate = AutoAlignstates.HAND_ALIGN;
                }
                AutoAlign = false;
                break;
            case HAND_ALIGN:
                if(!AutoAlign) { //run once
                    hp.HandAngle = 180 - pipeline.getDetectedAngle() + 90;
                    endEffector.hand(ServoRegulizer(hp.MStandard * (hp.HandAngle) + hp.BStandard));
                }
                AutoAlign = true;
                break;
        }//switch case end
    }//method end


    public void SampleGrabAuto() {
        endEffector.arm(0.52);
        endEffector.wrist(0.18);
//        if(AutoAlign){
//            tp.TurretAngle = (endEffector.Turret.getPosition() - tp.BStandard)*tp.MStandard;
//            endEffector.hand(ServoRegulizer(hp.MStandard * (-tp.TurretAngle) + hp.BStandard));
//        }
//        Camerastate = AutoAlignstates.TURRET_ALIGN;
//      AutoAlign = false;
    }




    //Common methods
    public void ClawClose() {
        endEffector.claw(0.24);
    }

    public void ClawOpen() {
        endEffector.claw(0.65);
    }

    public void InsideGrabPecked() {
        endEffector.claw(0.75);
    }

    public void InsideGrabBeakOpen() {
        endEffector.claw(0.5);
    }















    //SAMPLE METHODS START HERE




    public void SampleHover() {
        double JoystickIncrement = 0;
        boxtube.setPivot(pivotHorizontal);
        endEffector.arm(0.46);
        endEffector.wrist(0.11);


        ClawOpen();


        //button detections
        if (gamepadOperator.triangle) {
            wasPressedTriangle = true;
        }

        if (gamepadOperator.dpad_down) {
            minExtendSubPressed = true;
            currentExtension = minExtension;
        }
        if (gamepadOperator.dpad_left) {
            midExtendSubPressed = true;
            currentExtension = midExtension;
        }
        if (gamepadOperator.dpad_up) {
            maxExtendSubPressed = true;
            currentExtension = fullExtension;
        }

        //Falling edges here

        if (!gamepadOperator.triangle && wasPressedTriangle) {
            horizontal = !horizontal;
            //hand = (horizontal)? 0.5:0.17;
            hp.HandAngle = (horizontal)? 90 : 180;
            wasPressedTriangle = false;
        }
        if (!gamepadOperator.dpad_down && minExtendSubPressed) {
            minExtendSubPressed = false;
        }
        if (!gamepadOperator.dpad_left && lowExtendSubPressed) {
            lowExtendSubPressed = false;
        }
        if (!gamepadOperator.dpad_right && midExtendSubPressed) {
            midExtendSubPressed = false;
        }
        if (!gamepadOperator.dpad_down && maxExtendSubPressed) {
            maxExtendSubPressed = false;
        }

        if (gamepadOperator.left_bumper) {
            wasPressedL = true;
        }
        if (gamepadOperator.right_bumper) {
            wasPressedR = true;
        }

        if (!gamepadOperator.left_bumper && wasPressedL) {
          // hand+=0.1;
            hp.HandAngle +=30;
            wasPressedL = false;
        }
        if (!gamepadOperator.right_bumper && wasPressedR) {
           //hand-=0.1;
            hp.HandAngle -=30;
            wasPressedR = false;
        }
            JoystickIncrement = MainTele.JoyStickInc *gamepadOperator.left_stick_y;
        //boxutbe adjustment


        hand = ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle) + hp.BStandard); //getting the angle


       if(Math.abs(gamepadOperator.left_trigger) > 0){
           tp.TurretAngle = 90 + gamepadOperator.left_trigger * 30; //freecking wire offset
       }
       if (Math.abs(gamepadOperator.right_trigger) > 0){
           tp.TurretAngle = 90 + gamepadOperator.right_trigger * -20;
        }


        endEffector.turret(tp.MStandard*(tp.TurretAngle)+tp.BStandard);

        endEffector.hand(hand);

       boxtube.setExt(currentExtension-= JoystickIncrement);

        //Add stuff to move extension + hand stuff using new .copy() method
    }

    public void SampleGrab() {
        endEffector.arm(0.52);
        endEffector.wrist(0.18);
    }




    public void LoiterSample() {
        boxtube.setExt(0);
        endEffector.hand(0.5);
        endEffector.arm(0.5);
        endEffector.turret(0.55);
       ClawClose();
        endEffector.wrist(0.7);
    }

    public void PivotBack() {
        boxtube.setPivot(pivotBackPos);
        endEffector.hand(0.485);
        endEffector.arm(0.5);
        endEffector.turret(0.55);
        endEffector.wrist(0.6);
        hand = 0.5;//for sample hover
    }

    public void BasketExtension() { // Ready to score
        boxtube.setExt(basketExtension);
        boxtube.setPivot(pivotBackPos);
    }

    public void BasketScore() {
    tp.TurretAngle = 90;
        ClawOpen();
        endEffector.arm(0.5);
        endEffector.hand(0.485);
        endEffector.turret(0.55);
        endEffector.wrist(0.72); //flick position
    }

    public void ObsZoneScore() {
        ClawOpen();
    }

    public void BasketReturn() {
        boxtube.setExt(0);
        currentExtension = 0;
        boxtube.setPivot(pivotBackPos);
        endEffector.arm(0.5);
        endEffector.hand(0.17);
        endEffector.wrist(0.3);
    }

    public void obsZoneRelease() {
        boxtube.setPivot(pivotHorizontal);
        boxtube.setExt(fullExtension);
        endEffector.arm(0.47);
        endEffector.wrist(0.25);
        endEffector.hand(0.48);
        endEffector.turret(0.55);
        ClawClose();
    }











    //SPEC POSITIONS

    public void SpecimenWall() {
        boxtube.setExt(specimenWallExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.16);
        endEffector.wrist(0.34);
        endEffector.arm(0.34);
        endEffector.turret(0.55);
        ClawOpen();

    }
    public void SpecimenWallTele() {
        boxtube.setExt(0);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.16);
        endEffector.wrist(0.32);
        endEffector.arm(0.32);
        endEffector.turret(0.55);
        ClawOpen();

    }

    public void SpecimenWallGrab() {
        ClawClose();
        //drive.SoftReset(); make a teleop version
    }

    public void SpecimenWallUp() {
        boxtube.setExt(specimenWallExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.17);
        endEffector.wrist(0.45);
        endEffector.arm(0.32);
        endEffector.turret(0.55);
        ClawClose();
    }
    public void SpecimenWallUpTele() {
        boxtube.setExt(0);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.17);
        endEffector.wrist(0.45);
        endEffector.arm(0.32);
        endEffector.turret(0.55);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.setExt(0);
        boxtube.setPivot(pivotBackPos);
        endEffector.hand(0.8);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.wrist(0.25);
        ClawClose();
    }

    public void SpecimenPostScore() {

        boxtube.setExt(0);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.hand(0.2);
        endEffector.wrist(0.6);
        boxtube.setPivot(0);
        ClawOpen();


    }

}
