package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Module {

    private static Differential differential;

    public Module(TalonSRX top, TalonSRX bottom) {
        differential = new Differential(top, bottom);
    }
}