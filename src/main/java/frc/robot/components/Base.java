package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Base {
    /**
     * Class Base is a singleton class that takes joystick inputs and generates movement and 
     * rotation vectors for each module, which are then given to the corresponding module objects
     */

    /**
     * Singleton declaration
     */
    private static Base instance = null;

    public static Base getInstance() {
        if (instance == null) {
            instance = new Base();
        }
        return instance;
    }

    private static Module[] modules = new Module[4]; // Swerve modules in the order: SE, NE, NW, SW

    public Base(){
        int[] talons = Constants.TALON_IDS;

        for(int j = 0; j < 4; j++) {
            modules[j] = new Module(new TalonSRX(talons[j * 2]), new TalonSRX(talons[j * 2 + 1]));
            //a module objects are created, with the corresponding Talon ID's from the constant
        }
    }

    public Module getModule(int id) {
        return modules[id];
    }
    
    public static void directControl() {

    }

    public static void fieldOrientedControl() {

    }

    public void reset() {
        for(Module mod : modules) {
            mod.reset();
        }
    }
}