package Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EndEffector {

    public Servo Wrist, Arm1, Arm2, Turret, Hand, Claw;


    public EndEffector(HardwareMap hardwareMap) {

        Wrist = hardwareMap.get(Servo.class, "Servo0");
        Arm1 = hardwareMap.get(Servo.class, "Servo1");
        Arm2 = hardwareMap.get(Servo.class, "Servo2");
        Turret = hardwareMap.get(Servo.class, "Servo5");
        Hand = hardwareMap.get(Servo.class, "Servo3");
        Claw = hardwareMap.get(Servo.class, "Servo4");
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

public void wrist(double pos){Wrist.setPosition(pos);}

    public void arm(double ticks) {
        Arm1.setPosition(ticks);
        Arm2.setPosition(ticks);
    }

}
