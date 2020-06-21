package frc.robot;

public class Constants {

    public enum SwerveMode {
        POSITION,
        VELOCITY
    }

    /**
     * Differential carrier position PID (direction of the module)
     */
    public static final int PID_CARRIER_SLOT = 0;
    public static final double CARRIER_KF = 0.0;
    public static final double CARRIER_KP = 0.0;
    public static final double CARRIER_KI = 0.0;
    public static final double CARRIER_KD = 0.0;
    public static final int CARRIER_KIZ = 0;
    public static final double CARRIER_COEFFICIENT = 0.5;

    /**
     * Differential planet velocity PID (velocity of the wheel)
     */
    public static final int PID_PLANET_VELOCITY_SLOT = 1;
    public static final double PLANET_VELOCITY_KF = 0.0;
    public static final double PLANET_VELOCITY_KP = 0.0;
    public static final double PLANET_VELOCITY_KI = 0.0;
    public static final double PLANET_VELOCITY_KD = 0.0;
    public static final int PLANET_VELOCITY_KIZ = 0;
    public static final double PLANET_VELOCITY_SENSOR_COEFFICIENT = 0.5;

    public static final int[] TALON_IDS = {
        //top down ( clockwise motor, counterclockwise motor)
        0, 1, //SE module
        2, 3, //NE module
        4, 5, //NW module
        6, 7  //SW module
    };
}