package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import Subsystems.Drivetrain;

@Config
@TeleOp(name = "Way Point Tuner")
public class WayPointingTuner extends LinearOpMode {

    FtcDashboard dashboard;
    MultipleTelemetry tele;
     ElapsedTime time;

     public static int State = 0;

     Drivetrain drive;

     public static double KpX = -0.1, KpY = -0.17, KpTheta = -0.023, KdX = -0.02, KdY = -0.02,
    targetHeading = 0,targetX = 0,targetY = 0;

     private boolean wasPressedGampadA = false;

     public static boolean score = false;
     private double lastErrorx,lastErrory,powerX,powerY;

     public static double specScorex = 0,specScorey = 0, pickupx = 0,pickupy = 0;


    @Override
        public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        drive = new Drivetrain(hardwareMap);

        while (opModeInInit()){
            powerX = 0;
            powerY = 0;
            //reseting exept for in state
            tele.update();
            drive.update();
            if(gamepad1.options){drive.Reset();}

            switch(State){
                case 0:
                    tele.addLine("Switch to state 1 for localization test make sure offsets are set in drivetrain");
                    break;
                case 1:
                    tele.addData("X offest:", drive.getPos()[0]);
                    tele.addData("Y offest:", drive.getPos()[1]);
                    tele.addData("Heading offset:", drive.getPos()[2]);
                    break;
                case 2:
                    tele.addLine("Heading PID");
                    tele.addData("Current Heading:", drive.getPos()[2]);
                    tele.addData("Target Heading",targetHeading);
                    drive.TeleopControl(0,0,KpTheta*(targetHeading - drive.getPos()[2]));
                    break;
                case 3:
                    tele.addLine("X PID (guessed tele-op carfull!!!)");
                    tele.addData("Current X", drive.getPos()[0]);
                    tele.addData("Target X",targetX);
                    powerX = KpX*(targetX - drive.getPos()[0]) + KdX*(targetX - drive.getPos()[0] - lastErrorx)/time.seconds();
                    time.reset();
                    lastErrorx = targetX - drive.getPos()[0];
                    drive.TeleopControl(powerX,0,0);
                    break;
                case 4:
                    tele.addLine("Y PID (guessed tele-op carfull!!!)");
                    tele.addData("Current Y", drive.getPos()[1]);
                    tele.addData("Target Y",targetY);
                    powerY = KpY*(targetY - drive.getPos()[1]) + KdY*(targetY - drive.getPos()[1] - lastErrory)/time.seconds();
                    time.reset();
                    lastErrory = targetY - drive.getPos()[1];
                    drive.TeleopControl(0,powerY,0);
                    break;
                case 5:
                    tele.addLine("Full tester");
                    powerX = KpX*(targetX - drive.getPos()[0]) + KdX*(targetX - drive.getPos()[0] - lastErrorx)/time.seconds();
                    lastErrorx = targetX - drive.getPos()[0];
                    powerY = KpY*(targetY - drive.getPos()[1]) + KdY*(targetY - drive.getPos()[1] - lastErrory)/time.seconds();
                    lastErrory = targetY - drive.getPos()[1];
                    drive.TeleopControl(powerX,powerY,KpTheta*(targetHeading - drive.getPos()[2]));
                    time.reset();
                    break;
                case 6:
                    tele.addLine("Full tester but wayPointing (The Pick up pose should be 0,0)");
                    if(gamepad1.options){drive.Reset();}

                    if(gamepad1.a){wasPressedGampadA = true;}

                    if (!gamepad1.a && wasPressedGampadA){
                        targetX=(score)? specScorex : pickupx;
                        targetY=(score)? specScorey : pickupy;
                        wasPressedGampadA = false;
                    }

                        powerX = KpX * (targetX - drive.getPos()[0]) + KdX * (targetX - drive.getPos()[0] - lastErrorx) / time.seconds();
                        time.reset();
                        lastErrorx = targetX - drive.getPos()[0];
                        powerY = KpY * (targetY - drive.getPos()[1]) + KdY * (targetY - drive.getPos()[1] - lastErrory) / time.seconds();
                        time.reset();
                        lastErrory = targetY - drive.getPos()[1];
                        drive.TeleopControl(powerX, powerY, KpTheta * (targetHeading - drive.getPos()[2]));


            } //switch end
        } //inint end


        waitForStart();


} //run opmode end
}
