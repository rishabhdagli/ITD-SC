package Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Boxtube {

    public static double PivotDownKp = 0.0008, PivotDownKd = 0.00011, PivotkP = 0.002, PivotKd = 0.0001, Tick90 = 1086, FF = 0.088, period = (2 * Math.PI) / (Tick90 * 4), lasterror,
            upkP = 0.0002, downkD = 0.00001, downkP = 0.0003, horizantalkP=0.00015;
    final int MaxExtension = 57000;
    public double targetPiv, targetExt;
    double currentPivot,currentBoxtube;
    public DcMotorEx Pivot, BT1, BT2, BT3;
    public double offsetAngle, ExtPwr;
    Telemetry t;
    HardwareMap h;
    AnalogInput PivotAbs;
    AnalogInput boxtubeAbs;
    ElapsedTime Pivottimer,Extensiontimer;

    public Boxtube(HardwareMap hardwareMap, int x) {
        // Initialize motors with proper names
        Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");
        BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        BT2 = hardwareMap.get(DcMotorEx.class, "Boxtube2odoleft");
        BT3 = hardwareMap.get(DcMotorEx.class, "Boxtube3odoright");

        //PivotEnc = hardwareMap.get(DcMotorEx.class, "Pivot");

        // Set to run without encoders
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Correct motor directions if needed
        Pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        //theese are good
        BT1.setDirection(DcMotorSimple.Direction.FORWARD);
        BT2.setDirection(DcMotorSimple.Direction.REVERSE);
        BT3.setDirection(DcMotorSimple.Direction.REVERSE);

        Pivottimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Pivottimer.startTime();

        Extensiontimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Extensiontimer.startTime();

        PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        boxtubeAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");

    }

    public Boxtube(HardwareMap hardwareMap) {
        // Initialize motors with proper names
        Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");
        BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        BT2 = hardwareMap.get(DcMotorEx.class, "Boxtube2odoleft");
        BT3 = hardwareMap.get(DcMotorEx.class, "Boxtube3odoright");

        //PivotEnc = hardwareMap.get(DcMotorEx.class, "Pivot");

        // Reset encoders
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoders
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Correct motor directions if needed
        Pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        //theese are good
        BT1.setDirection(DcMotorSimple.Direction.FORWARD);
        BT2.setDirection(DcMotorSimple.Direction.REVERSE);
        BT3.setDirection(DcMotorSimple.Direction.REVERSE);

        Pivottimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Pivottimer.startTime();

        Extensiontimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        Extensiontimer.startTime();

      //  PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
       // boxtubeAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");
    }


    public void update() {
        updatePiv();
        updateExt();
    }

    public void changekP(double kp) {
        PivotkP = kp;
    }


    public void setPivot(double val) {
        targetPiv = val;
    }

    public void updatePiv() {
        PivotMove(targetPiv);
    }

    public int getExtpos() {
        return BT1.getCurrentPosition();
    }

    public int getPivpos() {
        return Pivot.getCurrentPosition();
    }


    public void PivotMove(double targetPos) {
         currentPivot =  -Pivot.getCurrentPosition();
        double error = targetPos - currentPivot;

        if (error > 0) {
            double power = PivotkP * error + PivotKd * (error - lasterror) / Pivottimer.seconds() + FF * Math.cos(period * Pivot.getCurrentPosition());
            Pivot.setPower(power);
            lasterror = error;

        } else if (error < 0) {
            double power = PivotDownKp * error + PivotDownKd * (error - lasterror) / Pivottimer.seconds();
            Pivot.setPower(power);
            lasterror = error;
        }
        Pivottimer.reset();
    }

    public void ExtensionPower(double power) {
        BT1.setPower(power);
        BT2.setPower(power);
        BT3.setPower(power);
    }

    public void setExt(double val) {
        targetExt = val;
    }

    public void updateExt() {
        ExtensionMove(targetExt);
    }


    //for slowing when moving
    public boolean PivotisMoving() {
        //change to if moving slow when down
        return Pivot.getPower() > 0.3;
    }


    public void ExtensionMove(double extensionTargetPos) {
         currentBoxtube = BT1.getCurrentPosition();
        double extensionError = (extensionTargetPos) + BT1.getCurrentPosition();

        if(currentPivot > Tick90/4.0 && extensionError > 0){
            ExtPwr  = (extensionError* upkP);}
        else if(currentPivot > Tick90/4.0 && extensionError < 0){
            ExtPwr  = (extensionError* downkP) + (downkD *((extensionError - lastExtError)/ExentionTimer.seconds()));}
        else if (currentPivot < Tick90/4.0){
            ExtPwr = (extensionError * horizantalkP);}
        else { power = 0;}
        ExtensionPower(ExtPwr);

    }
}
