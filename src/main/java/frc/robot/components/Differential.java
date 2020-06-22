package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.Constants;
import frc.robot.Constants.SwerveMode;

public class Differential {

    private SwerveMode mode = SwerveMode.VELOCITY;

    private TalonSRX top; // the motor that drives the top bevel gear
    private TalonSRX bottom; // the motor that drives the bottom bevel gear

    public Differential(TalonSRX t, TalonSRX b) { 
        top = t;
        bottom = b;
        basicSetup(top, false);
        basicSetup(bottom, true);
        
        System.out.println("new diff:" + top.getDeviceID()+bottom.getDeviceID());

        //use cw motor for closed loop setup
        closedLoopVeloSetup(top, bottom);
        zero();
    }

    private void basicSetup(TalonSRX motor, boolean invert) {
        System.out.println(motor.configFactoryDefault());
        motor.set(ControlMode.PercentOutput, 0);
        motor.setSensorPhase(false);
        motor.setInverted(invert);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * closedLoopVeloSetup is the setup routine for velocity based control in teleop
     * 
     * the two PID loops are configured so that the inputs are the sum and difference of the 
     * encoder values
     * The top master runs the sum of the two PID loops, and the bottom master runs the difference of
     * the two loops
     * 
     * displacement = PID0((pos1 - pos2) / 2);
     * direction = PID1((pos1 + po2) / 2);
     * motor0 = PID1 + PID0;
     * motor1 = PID1 - PID0;
     * @param master the master to be configured
     */
    private void closedLoopVeloSetup(TalonSRX master, TalonSRX remote) {

        // setup the encoder on the other motor as an input
        remote.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Relative, 
            Constants.REMOTE_SENSOR_SLOT, Constants.TIMEOUT_MS);

        master.configRemoteFeedbackFilter(
            remote.getDeviceID(),
            RemoteSensorSource.TalonSRX_SelectedSensor, Constants.REMOTE_SENSOR_SLOT, 
            Constants.TIMEOUT_MS);

        /**
         * displacement (velocity) PID [PID loop 0]
         * sets up the equation:
         * displacement = (pos1 - pos2) / 2
         */

        // setup the sensor addition and the coefficient
        master.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, 
            Constants.TIMEOUT_MS);
            master.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, 
                Constants.TIMEOUT_MS);
        master.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 
            Constants.PID_PLANET_VELOCITY_SLOT, Constants.TIMEOUT_MS);
        master.configSelectedFeedbackCoefficient(Constants.PLANET_VELOCITY_SENSOR_COEFFICIENT, 
            Constants.PID_PLANET_VELOCITY_SLOT, Constants.TIMEOUT_MS);

        // setup the constants
        master.config_kF(Constants.PID_PLANET_VELOCITY_SLOT, Constants.PLANET_VELOCITY_KF, // F
        Constants.TIMEOUT_MS);
        master.config_kP(Constants.PID_PLANET_VELOCITY_SLOT, Constants.PLANET_VELOCITY_KP, // P
            Constants.TIMEOUT_MS);
        master.config_kI(Constants.PID_PLANET_VELOCITY_SLOT, Constants.PLANET_VELOCITY_KI, // I
            Constants.TIMEOUT_MS);
        master.config_kD(Constants.PID_PLANET_VELOCITY_SLOT, Constants.PLANET_VELOCITY_KD, // D
            Constants.TIMEOUT_MS);
        master.config_IntegralZone(Constants.PID_PLANET_VELOCITY_SLOT, // IZ
        Constants.PLANET_VELOCITY_KIZ, Constants.TIMEOUT_MS);
        /**
         * module direction (position) PID [PID loop 1]
         * sets up the equation:
         * direction = (pos1 + po2) / 2
         */

        // setup the sensor addition and the coefficient
        master.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.CTRE_MagEncoder_Relative, 
        Constants.TIMEOUT_MS);
        master.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.RemoteSensor0, 
            Constants.TIMEOUT_MS);
        master.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 
            Constants.PID_CARRIER_SLOT, Constants.TIMEOUT_MS);
        master.configSelectedFeedbackCoefficient(Constants.CARRIER_COEFFICIENT, 
            Constants.PID_CARRIER_SLOT, Constants.TIMEOUT_MS);

        // setup the constants
        master.config_kP(Constants.PID_CARRIER_SLOT, Constants.CARRIER_KP, // P
            Constants.TIMEOUT_MS);
        master.config_kI(Constants.PID_CARRIER_SLOT, Constants.CARRIER_KI, // I
            Constants.TIMEOUT_MS);
        master.config_kD(Constants.PID_CARRIER_SLOT, Constants.CARRIER_KD, // D
            Constants.TIMEOUT_MS);
        master.config_IntegralZone(Constants.PID_CARRIER_SLOT, // IZ
            Constants.CARRIER_KIZ, Constants.TIMEOUT_MS);

        remote.follow(master, FollowerType.AuxOutput1);

        mode = SwerveMode.VELOCITY;
    }

    public int veloControl(double speed, double targetDifference) {
        if(mode == SwerveMode.VELOCITY){
            // even when using the Velocity control mode, the aux PID runs in position
            top.set(ControlMode.Velocity, speed, DemandType.AuxPID, targetDifference);
            System.out.println(top.getSelectedSensorPosition(1));
            return 0;
        }
        
        System.out.println("velocity control was attempted in the wrong mode");
        closedLoopVeloSetup(top, bottom);
        return 1;
    }

    public void zero() {
        zeroTop();
        zeroBottom();
    }

    public void zeroTop() {
        top.setSelectedSensorPosition(0);
    }

    public void zeroBottom() {
        bottom.setSelectedSensorPosition(0);
    }

    public double getRotation() {
        return top.getSelectedSensorPosition(Constants.PID_CARRIER_SLOT);
    }

    public double getDisplacement() {
        return top.getSelectedSensorPosition(Constants.PID_PLANET_POSITION_SLOT);
    }

    public void resetMotors() {
        top.set(ControlMode.PercentOutput, 0);
        bottom.set(ControlMode.PercentOutput, 0);
    }

}