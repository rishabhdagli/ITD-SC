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
    Drivetrain drive;



    //PIVOT VARIABLES

    //over shoort 1120 to ensure locking
    double pivotBackPos = 1280, pivotHorizontal = 0, pivotSpecSpos = 900, pivotPreLoad = 550;


    //EXTENSION VARIABLES
    double minExtension = 000, midExtension = 12500, fullExtension = 25000, basketExtension = 56000, specScoreExtension = 15250, currentExtension = minExtension, specimenWallExtension = 15000;


    //For Endeffecotor
    double hand = 0.5;

    //BOOLEANS FOR BUTTONS
    boolean wasPressedL, wasPressedR, minExtendSubPressed, midExtendSubPressed, lowExtendSubPressed, maxExtendSubPressed,SampleHandHover = false;
    //Gamepads
    Gamepad gamepadOperator, gamepadDriver;
    //VISION
    private CrushSampleAnglePipelineTurretTrial pipeline;
    private VisionPortal VP;


    // Constructors for auto-robot and tele-robot


    public Robot(HardwareMap hardwareMap, Gamepad g1, Gamepad g2) {

        drive = new Drivetrain(hardwareMap);
        boxtube = new Boxtube(hardwareMap, 1); //init without reseting 0
        endEffector = new EndEffector(hardwareMap);

        gamepadOperator = g2;
        gamepadDriver = g1;
    }

    public Robot(HardwareMap h, Pose startPose) {

        Constants.setConstants(FConstants.class, LConstants.class);

        boxtube = new Boxtube(h);
        endEffector = new EndEffector(h);
        follower = new Follower(h);

        currentExtension = boxtube.getExtpos();

        follower.setStartingPose(startPose);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        pipeline = new CrushSampleAnglePipelineTurretTrial();
//        VP = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

    }


    //TELE - Op drive control
    public void TeleControl(double yMultiplier, double xMultiplier, double rxMultiplier) {
        drive.TeleopControl(yMultiplier * gamepadDriver.left_stick_y, xMultiplier * gamepadDriver.left_stick_x, rxMultiplier * gamepadDriver.right_stick_x);
    }


    //AUTON SPECIFIC METHOD HERE
    //TODO: NEED TO REDO ALL OF THESE METHODS
    public void InitPosition() {
        endEffector.turret(0.2);
        endEffector.hand(47);
        boxtube.setPivot(200);
        boxtube.setExt(0); //less than min
        ClawClose();
    }

    public void SpecimenPreLoad() {
        endEffector.hand(0.16);
        endEffector.wrist(0.25);
        endEffector.turret(0.55);
        endEffector.turret(0.47);
        boxtube.setPivot(pivotPreLoad);
        boxtube.setExt(0);
        ClawClose();
    }

    public void PreloadSpecExt() {
        endEffector.hand(0.16);
        endEffector.wrist(0.25);
        endEffector.turret(0.55);
        endEffector.turret(0.47);
        boxtube.setPivot(pivotPreLoad);
        boxtube.setExt(13750);
        ClawClose();

    }

    public void SpecimenPreLoadScore() {
        endEffector.hand(0.17);
        //change in Method Loiter In aswell
        endEffector.arm(0.47);

        boxtube.setPivot(50);
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






    //Common methods
    public void ClawOpen() {
        endEffector.claw(0.4);
    }

    public void ClawClose() {
        endEffector.claw(0.71);
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
        endEffector.arm(0.5);
        endEffector.wrist(0.13);
        endEffector.turret(0.55);

        InsideGrabPecked();

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
           hand+=0.05;
            wasPressedL = false;
        }
        if (!gamepadOperator.right_bumper && wasPressedR) {
           hand-=0.05;
            wasPressedR = false;
        }
            JoystickIncrement = MainTele.JoyStickInc *gamepadOperator.left_stick_y;

        endEffector.hand(hand);

        boxtube.setExt(currentExtension-= JoystickIncrement);

        //Add stuff to move extension + hand stuff using new .copy() method
    }

    public void SampleGrab() {
        endEffector.arm(0.55);
        endEffector.wrist(0.16);
        InsideGrabPecked();
    }

    public void LoiterSample() {
        boxtube.setExt(0);
        endEffector.hand(0.5);
        endEffector.arm(0.5);
        endEffector.turret(0.55);
        InsideGrabBeakOpen();
        endEffector.wrist(0.7);
    }

    public void PivotBack() {
        boxtube.setPivot(pivotBackPos);
        endEffector.hand(0.17);
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
        //flicks the sample inside high basket
        InsideGrabPecked();
        endEffector.arm(0.5);
        endEffector.hand(0.17);
        endEffector.turret(0.55);
        endEffector.wrist(0.72); //flick position
    }

    public void ObsZoneScore() {
        InsideGrabPecked();
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
        endEffector.arm(0.36);
        endEffector.turret(0.55);
        ClawOpen();

    }

    public void SpecimenWallGrab() {
        ClawClose();
    }

    public void SpecimenWallUp() {
        boxtube.setExt(specimenWallExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.17);
        endEffector.wrist(0.4);
        endEffector.arm(0.36);
        endEffector.turret(0.55);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.setExt(0);
        boxtube.setPivot(pivotBackPos);
        endEffector.hand(0.8);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.wrist(0.27);
        ClawClose();
    }

    public void SpecimenPostScore() {

        boxtube.setExt(0);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.hand(0.8);

        endEffector.wrist(0.5);
        boxtube.setPivot(0);
        ClawOpen();


    }

}
