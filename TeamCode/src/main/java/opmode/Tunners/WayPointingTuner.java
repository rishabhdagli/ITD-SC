package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import Subsystems.Drivetrain;

public class WayPointingTuner extends LinearOpMode {

    FtcDashboard dashboard;
    MultipleTelemetry tele;
     ElapsedTime time;

     public static int State = 0;

     Drivetrain drive;

     public static double KpX = 0, KpY = 0, KpTheta = 0, KdX = 0, KdY = 0, KdTheta = 0;

    @Override
        public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        drive = new Drivetrain(hardwareMap);

        while (opModeInInit()){
            tele.update();
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


                    break;
            } //switch end
        } //inint end


        waitForStart();


} //run opmode end
}
