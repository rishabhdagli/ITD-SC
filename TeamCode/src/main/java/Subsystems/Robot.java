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

import opmode.Vision.CrushSampleAnglePipelineTurretTrial;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Robot {

    //COMMON
    ElapsedTime timer;
    public Boxtube boxtube;
    public EndEffector endEffector;
    Drivetrain drive;




    //PIVOT VARIABLES
     double pivotBackPos = 1120,pivotHorizontal = 0, pivotSpecSpos = 900
            ,pivotPreLoad = 550;



    //EXTENTION VARIABLES
    double minExtension = 2000,midExtension = 10000,fullExtension = 25000,
            basketExtension = 40000,specScoreExtension = 15250,
            currentExtension = midExtension,specimenWallExtension = 7000;



    //BOOLEANS FOR BUTTONS
    boolean wasPressedL,wasPressedR, minExtendSubPressed, midExtendSubPressed,
            lowExtendSubPressed, maxExtendSubPressed;



    //Gamepads
    Gamepad gamepadOperator, gamepadDriver;

    //Pedro
    public Follower follower;


    //VISION
    private CrushSampleAnglePipelineTurretTrial pipeline;
    private VisionPortal VP;













    //Contructors for that merge autorobot and telerobot


    public Robot(HardwareMap hardwareMap, Gamepad g1, Gamepad g2) {

        drive = new Drivetrain(hardwareMap);
        boxtube = new Boxtube(hardwareMap, 1);
        endEffector = new EndEffector(hardwareMap);

        gamepadOperator = g2;
        gamepadDriver = g1;
    }

    public Robot(HardwareMap h, Pose startPose) {

        Constants.setConstants(FConstants.class, LConstants.class);

        boxtube = new Boxtube(h);
        endEffector = new EndEffector(h);
        follower = new Follower(h);

        follower.setStartingPose(startPose);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        pipeline = new CrushSampleAnglePipelineTurretTrial();
        VP = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

    }








//TELE - Op drive control
    public void TeleControl(double yMultiplier, double xMultiplier, double rxMultiplier) {
        drive.TeleopControl(
                yMultiplier * gamepadDriver.left_stick_y,
                xMultiplier * gamepadDriver.left_stick_x,
                rxMultiplier * gamepadDriver.right_stick_x);
    }









    //AUTON SPECIFIC METHOD HERE
    //TODO: NEED TO REDO ALL OF THESE METHODS
    public void InitPosition(){
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

    public void PreloadSpecExt(){
        endEffector.hand(0.16);
        endEffector.wrist(0.25);
        endEffector.turret(0.55);
        endEffector.turret(0.47);
        boxtube.setPivot(pivotPreLoad);
        boxtube.setExt(13750);
        ClawClose();

    };

    public void SpecimenPreLoadScore(){
        endEffector.hand(0.16);
        //change in Method Loiter In aswell
        endEffector.arm(0.47);

        boxtube.setPivot(50);
        ClawClose();
    }
    public void SpecimenWallForSamplePush() {
        boxtube.ExtensionMove(minExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.165);
        endEffector.wrist(0.45);
        endEffector.arm(0.4);
        endEffector.turret(0.55);
        ClawOpen();

    }









    //Common methods
    public void ClawOpen() {
        endEffector.claw(0.46);
    }

    public void ClawClose() {
        endEffector.claw(0.3);
    }
    public void InsideGrabPecked() {
        endEffector.claw(0.27);
    }
    public void InsideGrabBeakOpen() {
        endEffector.claw(0.55);
    }

















    //SAMPLE METHODS START HERE

    public void Loiter() {
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(minExtension);
        endEffector.hand(0.48);
        endEffector.turret(0.55);
        ClawOpen();
    }



    public void SampleHover() {
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(currentExtension);
        endEffector.arm(0.47);
        endEffector.wrist(0.25);
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
            endEffector.hand(endEffector.handPos() + 0.05);
            wasPressedL = false;
        }
        if (!gamepadOperator.right_bumper && wasPressedR) {
            endEffector.hand(endEffector.handPos() - 0.05);
            wasPressedR = false;
        }

        //Add stuff to move extension + hand stuff using new .copy() method
    }

    public void SampleGrab() {
        endEffector.arm(0.55);
        endEffector.wrist(0.27);
        endEffector.hand(0.5);
        endEffector.turret(0.55);
    }

    public void LoiterSample() {
        boxtube.ExtensionMove(0);
        endEffector.hand(0.5);
        endEffector.arm(0.4);
        endEffector.turret(0.55);
        endEffector.claw(0.55);
        endEffector.wrist(0.7);
    }

    public void PivotBack() {
        boxtube.PivotMove(pivotBackPos);
        endEffector.hand(0.17);
        endEffector.arm(0.5);
        endEffector.turret(0.55);
        endEffector.claw(0.55);
        endEffector.wrist(0.65);
    }

    public void BasketScore() {
        InsideGrabPecked();
        endEffector.arm(0.5);
        endEffector.hand(0.17);
        endEffector.turret(0.55);
        endEffector.wrist(0.72);
    }

    public void ObsZoneScore(){
        InsideGrabPecked();
    }

    public void BasketExtension() { // Ready to score
        boxtube.ExtensionMove(basketExtension);
    }

    public void BasketReturn() {
        boxtube.ExtensionMove(0) ;
        endEffector.arm(0.5);
        endEffector.claw(0.27);
        endEffector.hand(0.17);
        endEffector.wrist(0.3);
    }

    public void obsZoneRelease() {
        boxtube.PivotMove(pivotHorizontal);
        boxtube.ExtensionMove(fullExtension);
        endEffector.arm(0.47);
        endEffector.wrist(0.25);
        endEffector.hand(0.48);
        endEffector.turret(0.55);
        ClawClose();
    }













    //SPEC POSITIONS

    public void SpecimenWall() {
        boxtube.ExtensionMove(specimenWallExtension);
        boxtube.setPivot(pivotHorizontal);
        endEffector.hand(0.165);
        endEffector.Wrist.setPosition(0.45);
        //        endEffector.setEndEffector(60,-45);
        endEffector.arm(0.4);
        endEffector.turret(0.55);
        ClawOpen();

    }

    public void SpecimenWallGrab() {
        ClawClose();
    }

    public void SpecimenWallUp() {
        boxtube.ExtensionMove(0);
        boxtube.PivotMove(pivotBackPos);
        endEffector.hand(0.17);
        endEffector.turret(0.55);
        endEffector.arm(0.4);
        endEffector.wrist(0.45);
        ClawClose();
    }

    public void SpecimenPreScore() {
        boxtube.ExtensionMove(0);
        boxtube.PivotMove(pivotBackPos);
        endEffector.hand(0.8);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.wrist(0.3);
        ClawClose();
    }

    public void SpecimenPostScore() {

        boxtube.ExtensionMove(0);
        endEffector.turret(0.55);
        endEffector.arm(0.23);
        endEffector.hand(0.8);

        endEffector.wrist(0.5);
        boxtube.PivotMove(0);
        ClawOpen();


    }

}
