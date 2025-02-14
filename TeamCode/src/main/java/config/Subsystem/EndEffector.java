package config.Subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EndEffector {

    public Servo Wrist, Arm, Turret, Hand, Claw;

    HardwareMap h;

    Telemetry t;

    public EndEffector(HardwareMap hardwareMap, Telemetry t) {
        this.h = hardwareMap;
        this.t = t;

        Wrist = hardwareMap.get(Servo.class, "Servo6");
        Arm = hardwareMap.get(Servo.class, "Servo7");
        Turret = hardwareMap.get(Servo.class, "Servo10");
        Hand = hardwareMap.get(Servo.class, "Servo8");
        Claw = hardwareMap.get(Servo.class, "Servo9");
    }

    public void hand(double pos) {
        Hand.setPosition(pos);
    }

    public double handPos(){
        return Hand.getPosition();
    }

    public void turret(double pos) {
        Turret.setPosition(pos);
    }

    public void claw(double pos) {
        Claw.setPosition(pos);
    }

    public void setEndEffector(double armAngle, double wristAngle){
        double armTicks = 0.0033333*(armAngle) + 0.5;

        double offsetAngle = 391.30435 * (armTicks) - 195.65217;

        double wristTicks = -0.0031111*(wristAngle+25+offsetAngle) + 0.5;

        Arm.setPosition(armTicks);
        Wrist.setPosition(wristTicks);
    }
}
