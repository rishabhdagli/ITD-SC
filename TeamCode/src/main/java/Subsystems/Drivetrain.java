package Subsystems;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    public DcMotorEx LF,LR,RF,RR;

    GoBildaPinpointDriver PinPoint;

    public Drivetrain(HardwareMap hardwareMap) {

            LF = hardwareMap.get(DcMotorEx.class,"leftFront");
            LR = hardwareMap.get(DcMotorEx.class,"leftBack");
            RF = hardwareMap.get(DcMotorEx.class,"rightFront");
            RR = hardwareMap.get(DcMotorEx.class,"rightBack");


            //this must come before the run without encoder
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // correct motor directions for Crush
            LF.setDirection(DcMotorSimple.Direction.REVERSE);
            LR.setDirection(DcMotorSimple.Direction.REVERSE);

            PinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
            PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            PinPoint.resetPosAndIMU();


    }//init end

    public void TeleopControl(double y, double x, double rx) {
        y = -y; // Remember, Y stick value is reversed
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = rx * 0.75;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        //Right front and left front motors encoder are reversed


        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);

        PinPoint.update();
    }





}
