package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
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

        //use cw motor for closed loop setup
        closedLoopVeloSetup(top, bottom);
        zero();
    }

    private void basicSetup(TalonSRX motor, boolean invert) {
        motor.configFactoryDefault();
        motor.setInverted(invert);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
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
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        /**
         * displacement (velocity) PID [PID loop 0]
         * sets up the equation:
         * displacement = (pos1 - pos2) / 2
         */

        // setup the encoder on the other master as an input
        config.remoteFilter0.remoteSensorDeviceID = remote.getDeviceID();
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;

        // setup the sensor addition and the coefficient
        config.diff0Term = FeedbackDevice.CTRE_MagEncoder_Relative;
        config.diff0Term = FeedbackDevice.RemoteSensor0;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference;
        config.primaryPID.selectedFeedbackCoefficient = 
            Constants.PLANET_VELOCITY_SENSOR_COEFFICIENT;

        // setup the constants
        config.slot0.kP = Constants.PLANET_VELOCITY_KP;
        config.slot0.kI = Constants.PLANET_VELOCITY_KI;
        config.slot0.kD = Constants.PLANET_VELOCITY_KD;
        config.slot0.integralZone = Constants.PLANET_VELOCITY_KIZ;
        
        /**
         * module direction (position) PID [PID loop 1]
         * sets up the equation:
         * direction = (pos1 + po2) / 2
         */

        // setup the encoder on the other master as an input
        config.remoteFilter0.remoteSensorDeviceID = remote.getDeviceID();
        config.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;
        
        // setup the sensor addition and the coefficient
        config.sum0Term = FeedbackDevice.CTRE_MagEncoder_Relative;
        config.sum1Term = FeedbackDevice.RemoteSensor0;
        config.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum;
        config.auxiliaryPID.selectedFeedbackCoefficient = Constants.CARRIER_COEFFICIENT;

        // setup the constants
        config.slot1.kP = Constants.CARRIER_KP;
        config.slot1.kI = Constants.CARRIER_KI;
        config.slot1.kD = Constants.CARRIER_KD;
        config.slot1.kD = Constants.CARRIER_KF;
        config.slot1.integralZone = Constants.CARRIER_KIZ;

        /**
         * setup primary and aux output so that
         * motor0 = PID1 + PID0;
         * and
         * motor1 = PID1 - PID0;
         */
        config.auxPIDPolarity = false;

        master.configAllSettings(config);

        remote.follow(master, FollowerType.AuxOutput1);

        mode = SwerveMode.VELOCITY;
    }

    public int veloControl(double speed, double targetDifference) {
        if(mode == SwerveMode.VELOCITY){

            // even when using the Velocity control mode, the aux PID runs in position
            top.set(ControlMode.Velocity, targetDifference, DemandType.AuxPID, targetDifference);
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

}