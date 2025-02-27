package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Absolute encoder testing")
@Config
public class AbsoluteEnconder extends LinearOpMode {

    public static double m = 0,b=0;

    // Get analog port instance from hardwareMap
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dash = FtcDashboard.getInstance();

        AnalogInput PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        AnalogInput BTAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");
        DcMotorEx BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        DcMotorEx Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");

        BT1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BT1.setDirection(DcMotorSimple.Direction.REVERSE);

        BT1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive()) {

            double currentPivot = -Pivot.getCurrentPosition();
            double currentBoxtube = -BT1.getCurrentPosition();





            telemetry.addData("Pivot voltage", PivotAbs.getVoltage());
            telemetry.addData("Connection data",PivotAbs.getConnectionInfo());
            telemetry.addData("pivot ticks raw", currentPivot);
           // telemetry.addData("Pivot with absolute", absolutepivot);
            telemetry.addData("Calcualted Pivot pos", m*(PivotAbs.getVoltage()) + b);

            telemetry.addData("Boxtube Voltage", BTAbs.getVoltage());
            telemetry.addData("Boxtube ticks raw", currentBoxtube);
           // telemetry.addData("Boxtube ticks Absolute",absoluteBoxutbe);
            telemetry.update();
        }
    }
}
