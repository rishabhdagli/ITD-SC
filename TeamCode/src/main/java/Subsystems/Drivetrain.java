package Subsystems;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

            PinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
            PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            //PinPoint.setOffsets();
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



    public void FeildCentric(double y, double x, double rx) {
        // Reverse y-axis to correct for joystick inversion
        y = -y;

        // Cube the inputs for finer control at low speeds
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = rx * 0.75;

        // Get the current heading in degrees from PinPoint (assuming getAngle() returns degrees)
        double heading = PinPoint.getHeading();
        // Convert heading to radians
        double theta = Math.toRadians(heading);

        // Rotate the joystick input by -theta to convert field-centric to robot-centric values.
        // Using the standard rotation transformation for a -theta rotation:
        double robotX = x * Math.cos(theta) + y * Math.sin(theta);
        double robotY = -x * Math.sin(theta) + y * Math.cos(theta);

        // Normalize the motor powers so none exceed 1
        double denominator = Math.max(Math.abs(robotY) + Math.abs(robotX) + Math.abs(rx), 1);
        double frontLeftPower  = (robotY + robotX + rx) / denominator;
        double backLeftPower   = (robotY - robotX + rx) / denominator;
        double frontRightPower = (robotY - robotX - rx) / denominator;
        double backRightPower  = (robotY + robotX - rx) / denominator;

        // Set motor powers accordingly
        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);

        // Update the PinPoint (odometry) for the latest heading and position
        PinPoint.update();
    }

    public double[] getPos(){
        PinPoint.update();
        double[] pos = new double[3];
        pos[0] = PinPoint.getPosX();
        pos[1] = PinPoint.getPosY();
        pos[2] = PinPoint.getHeading();
        return pos;
    }






}
