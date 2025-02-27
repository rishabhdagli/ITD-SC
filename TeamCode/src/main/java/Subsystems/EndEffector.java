package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EndEffector {

    public Servo Wrist, Arm1, Arm2, Turret, Hand, Claw;


    public EndEffector(HardwareMap hardwareMap) {

        Wrist = hardwareMap.get(Servo.class, "Servo6");
        Arm1 = hardwareMap.get(Servo.class, "Servo7");
        Arm2 = hardwareMap.get(Servo.class, "");
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

    private void arm(double ticks) {
        Arm1.setPosition(ticks);
        Arm2.setPosition(ticks);
    }
    public void setEndEffector(double armAngle, double wristAngle){
        double armTicks = 0.0033333*(armAngle) + 0.5;

        double offsetAngle = 391.30435 * (armTicks) - 195.65217;

        double wristTicks = -0.0031111*(wristAngle+25+offsetAngle) + 0.5;

        arm(armTicks);
        Wrist.setPosition(wristTicks);
    }




}
