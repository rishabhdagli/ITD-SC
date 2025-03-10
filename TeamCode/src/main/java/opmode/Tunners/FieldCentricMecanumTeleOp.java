package opmode.Tunners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Subsystems.Drivetrain;

@TeleOp(name = "waypoint test")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    Drivetrain d;
    @Override
    public void runOpMode() throws InterruptedException {
        d = new Drivetrain(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            d.FeildCentric(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }

}