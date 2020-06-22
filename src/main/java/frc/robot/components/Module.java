package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Module {

    private Differential differential;
    private double currentAngle = 0;
    private double currentSensPhase = 0;
    private int adjust = 0;

    public Module(TalonSRX top, TalonSRX bottom) {
        differential = new Differential(top, bottom);
    }

    public void veloControl(double velocity, double angle) {

        // get the current sensor phase, current angle, and the change in angle desired
        currentSensPhase = differential.getRotation();
        currentAngle = (currentSensPhase % Constants.SENSOR_COUNTS_PER_ROTATATION) 
            * Constants.SENSOR_TO_RADIANS_COEF;
        double difference = angle - currentAngle;

        // if the angle change is more than 180, turn 180 less and reverse the motor
        // if already adjusted, change the threshold
        double threshold = Math.PI + adjust * Math.PI; // threshold is 180 degrees
        if(difference > threshold) {
            adjust -= 1;
            difference -= Math.PI;
        } else if(difference < -threshold) {
            adjust += 1;
            difference += Math.PI;
        }
        if(adjust % 2 == 1) {
            velocity *= -1;
        }
        
        double targetSensPhase = difference + currentSensPhase;
        differential.veloControl(velocity, targetSensPhase);
    }

    public Differential getDifferential() {
        return differential;
    }

    public void reset() {
        differential.resetMotors();
    }
}