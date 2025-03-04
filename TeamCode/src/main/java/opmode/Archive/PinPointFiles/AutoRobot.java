//package opmode.Archive.PinPointFiles;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import Subsystems.Boxtube;
//import Subsystems.EndEffector;
//import opmode.Vision.CrushSampleAnglePipelineTurretTrial;
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//public class AutoRobot {
//
//    public static class TurretParams{ public double Turret=0.5,MStandard = 0.00311111,BStandard=0.19,TurretAngle= 90, AGain = 0.00000000106228, BGain = 0.00000144933, CGain = 0.000025861, error,ServoGain = 0;
//    }
//
//    public static class HandParams{ public double Hand=0.5,MStandard = 0.00377778,BStandard=0.16,HandAngle = 90;}
//
//    ElapsedTime timer;
//    boolean FirstTime = true;
//    private CrushSampleAnglePipelineTurretTrial pipeline;
//    private VisionPortal VP;
//
//    public DcMotorEx LF,LR,RF,RR;
//    public Boxtube boxtube;
//    public EndEffector endEffector;
//    double specPreScore = 0;
//    double minExtension = 2000;
//    double specScore = 16000;
//
//    double specGrabExt = 10000;
//    double sampleScore = 30500;
//    double samplePickupExt = 2000;
//    double pivotHorizontal = 0;
//    double pivotSpecScore = 900;
//    double pivotPreLoad = 550;
//    double pivotBackPos = 1200;
//
//    public Telemetry t;
//    public Follower follower;
//
//    Gamepad gamepad;
//
//    public AutoRobot(HardwareMap h, Pose startPose) {
//
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        boxtube = new Boxtube(h);
//        endEffector = new EndEffector(h);
//        follower = new Follower(h);
//
//        follower.setStartingPose(startPose);
//
//        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//        pipeline = new CrushSampleAnglePipelineTurretTrial();
//        VP = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);
//
//    }
//
//    public void EndEffectorAlign(){
//        // Call this methods
//        // Detects a sample
//        //Aligns to the sample until turret error is low while the hand is using inverse kinematics
//        //Possible boxtube align
//        //Then the hand aligns
//        //then grab
//        if(FirstTime){
//            FirstTime = false;
//            timer.reset();
//        }
//        else{
//            if(timer.seconds() < 2){
//                //Put full tester here from AutoAlign test here
//            }
//            else if(timer.seconds() < 3){
//                //Put the part where the hand aligns
//            }
//            else if(timer.seconds() < 5){
//                //pick up actions
//            }
//
//        }
//
//
//    }
//    public void ChangePivotKP(double kp){
//        boxtube.changekP(kp);
//    }
//
//
//    public void ClawOpen() {
//        endEffector.claw(0.55);
//    }
//
//    public void ClawClose() {
//        endEffector.claw(1);
//    }
//
//
//
//
//
//
//    //for legality
//    public void InitPosition(){
//        endEffector.turret(0.2);
//        endEffector.hand(47);
//        boxtube.setPivot(200);
//        boxtube.setExt(0);
//        ClawClose();
//    }
//
//
//
//    public void Loiter(){
//        endEffector.hand(0.48);
//        //Guessed values
//        endEffector.arm(0.5);
//        endEffector.wrist(0.5);
//        //guess values end
//        endEffector.turret(0.47);
//        boxtube.setPivot(0);
//        boxtube.setExt(0);
//        ClawClose();
//    }
//
//    public void LoiterIn() {
//        endEffector.hand(0.48);
//        endEffector.turret(0.47);
//        //guesed values (From pre load score)
//        endEffector.arm(0.47);
//        endEffector.wrist(0.25);
//        //gues valeus end
//        boxtube.setPivot(0);
//        boxtube.setExt(0);
//    }
//
//
//
//    //Specimin Methods here
//    public void SpecimenPreLoad() {
//        endEffector.hand(0.16);
//        endEffector.setEndEffector(0,-15);
//        endEffector.turret(0.47);
//        boxtube.setPivot(pivotPreLoad);
//        boxtube.setExt(0);
//        ClawClose();
//    }
//
//    public void PreloadSpecExt(){
//        endEffector.hand(0.16);
//        endEffector.setEndEffector(0,-15);
//        endEffector.turret(0.47);
//        boxtube.setPivot(pivotPreLoad);
//        boxtube.setExt(13750);
//        ClawClose();
//
//    };
//
//    public void SpecimenPreLoadScore(){
//        endEffector.hand(0.16);
//        //change in Method Loiter In aswell
//        endEffector.arm(0.47);
//
//        boxtube.setPivot(50);
//        ClawClose();
//    }
//
//    public void SpecimenWall(){
//        boxtube.setPivot(pivotHorizontal);
//        endEffector.hand(0.16);
//        endEffector.setEndEffector(75,-60);
//        endEffector.turret(0.47);
//        ClawOpen();
//        if(boxtube.getPivpos() < 150){
//            boxtube.setExt(10000);
//        }
//    }
//    public void SpecimenWallForSamplePush(){
//        boxtube.setPivot(pivotHorizontal);
//        boxtube.setExt(minExtension);
//        endEffector.hand(0.17);
//        endEffector.wrist(0.45);
//        endEffector.arm(0.4);
//        endEffector.turret(0.55);
//        ClawOpen();
//    }
//
//
//    public void SpecimenWallGrab() {
//        endEffector.hand(0.16);
//        endEffector.setEndEffector(75,-60);
//        endEffector.turret(0.47);
//        ClawClose();
//    }
//
//    public void SpecimenPreScore() {
//        boxtube.setExt(specPreScore);
//        endEffector.turret(0.47);
//        endEffector.hand(0.16);
//        endEffector.setEndEffector(45,55);
//        ClawClose();
//    }
//
//    public void SpecimenLatch() {
//        boxtube.setExt(specScore);
//    }
//
//    public void SpecimenExtDown()
//    {
//        boxtube.setExt(0);
//    }
//
//    public void PivotUpSpec(){
//        boxtube.setPivot(pivotSpecScore);
//    }
//
//
//    //SAMPLE POSITIONS
//
//
//    public void SampleHover(){
//        boxtube.setPivot(pivotHorizontal);
//        boxtube.setExt(samplePickupExt);
//        endEffector.setEndEffector(30, -120);
//        endEffector.turret(0.5);
//        ClawOpen();
//    }
//
//    public void SampleGrab() {
//        boxtube.setPivot(pivotHorizontal);
//        endEffector.setEndEffector(-15,-55);
//        ClawOpen();
//    }
//
//    public void SampleHoverExt(double ext, double turret, double hand){
//        boxtube.setExt(ext);
//        boxtube.setPivot(pivotHorizontal);
//        endEffector.setEndEffector(30, -120);
//        endEffector.turret(turret);
//        endEffector.hand(hand);
//    }
//
//    public void LoiterSample() {
//        boxtube.setPivot(pivotHorizontal); // pivot should be horizontal
//        boxtube.setExt(samplePickupExt); //Make sure this is always in
//        endEffector.setEndEffector(80, -110);
//        endEffector.hand(0.48);
//        endEffector.turret(0.47);
//        ClawClose();
//
//    }
//
//    public void PivotBack(){
//        boxtube.setPivot(pivotBackPos);
//        boxtube.setExt(samplePickupExt); //Make sure this is always in when rotating
//        endEffector.setEndEffector(-20,60); //Needs fixing
//        endEffector.hand(0.48);
//        endEffector.turret(0.47);
//        ClawClose();
//    }
//
//    public void BasketExtension(){ // Ready to score
//        boxtube.setPivot(pivotBackPos);
//        boxtube.setExt(sampleScore);
//        endEffector.setEndEffector(0,65); //Needs fixing
//        endEffector.hand(0.48);
//        endEffector.turret(0.47);
//        ClawClose();
//    }
//
//    public void AutonReZero(){
//        boxtube.setExt(0);
//        boxtube.setPivot(pivotHorizontal);
//        endEffector.setEndEffector(30, -120);
//        endEffector.turret(0.47);
//        endEffector.hand(0.48);
//    }
//
//    public void BasketReturn(){
//        endEffector.setEndEffector(-40,0);
//        endEffector.hand(0.48);
//        endEffector.turret(0.47);
//        boxtube.setExt(samplePickupExt);
//        boxtube.setPivot(pivotBackPos - 500);
//        ClawOpen();
//    }
//}
