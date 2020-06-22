package frc.robot;

public class Constants {

    public enum SwerveMode {
        POSITION,
        VELOCITY
    }

    public static final int TIMEOUT_MS = 10;
    public static final int REMOTE_SENSOR_SLOT = 0;

    public static final int[] TALON_IDS = {
        //top down ( clockwise motor, counterclockwise motor)
        0, 1, //SE module
        2, 3, //NE module
        4, 5, //NW module
        6, 7  //SW module
    };

    public static final double GEAR_RATIO = 5;
    public static final int SENSOR_RESOLUTION = 4096;
    public static final double SENSOR_COUNTS_PER_ROTATATION = SENSOR_RESOLUTION * GEAR_RATIO;
    public static final double SENSOR_TO_RADIANS_COEF = 2 * Math.PI / SENSOR_COUNTS_PER_ROTATATION;

    /**
     * Differential carrier position PID (direction of the module)
     */
    public static final int PID_CARRIER_SLOT = 1;
    public static final double CARRIER_KP = 1.0;
    public static final double CARRIER_KI = 0.0;
    public static final double CARRIER_KD = 0.0;
    public static final int CARRIER_KIZ = 0;
    public static final double CARRIER_COEFFICIENT = 0.5;

    /**
     * Differential planet velocity PID (velocity of the wheel)
     */
    public static final int PID_PLANET_VELOCITY_SLOT = 0;
    public static final double PLANET_VELOCITY_KP = 0;
    public static final double PLANET_VELOCITY_KI = 0.0;
    public static final double PLANET_VELOCITY_KD = 0.0;
    public static final double PLANET_VELOCITY_KF = 0.02;
    public static final int PLANET_VELOCITY_KIZ = 0;
    public static final double PLANET_VELOCITY_SENSOR_COEFFICIENT = 0.5;

    /**
     * differential planet displacement PID (displacement of the wheel)
     */
    
    public static final int PID_PLANET_POSITION_SLOT = 0;
    public static final double PLANET_POSITION_KF = 0.0;
    public static final double PLANET_POSITION_KP = 0.0;
    public static final double PLANET_POSITION_KI = 0.0;
    public static final double PLANET_POSITION_KD = 0.0;
    public static final int PLANET_POSITION_KIZ = 0;
    public static final double PLANET_POSITION_SENSOR_COEFFICIENT = 0.5;
}