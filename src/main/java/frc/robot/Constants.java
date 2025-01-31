// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IntakeConstants {

    public static final int intakeID = 51;
    public static final int IRSignalID = 0;
    public static final double autoMotorSpeed = 0.25;   //For the intake when run through intakeFwd(), intakeRev(), aka auton
    public static final double teleopMotorSpeed = 0.25;  //For when we control the robot with an actual controller. 
  }
  public static class autoConfigConstants {
    public static final double maxVelocityMPS = 3;
    public static final double maxVelocityMPSSq = 3;
    public static final double maxAngularVelocity = 540;
    public static final double maxAngularVelocitySq = 720;
  }
  public static class ClimberConstants {
    public static final int armMotorID = 50;
  }



  public static class ImportantPositions { //Holds all our important positions for pathplanner
    
    /*    Awesome graph detailing positions😊😊😊 (woah I can paste emojis)
     *    ________________.
     *   / LHPL           |
     *  /LHPR             |
     * |                  |
     * |     6 /  \ 5     |
     *>|    1 | ## | 4    |
     * |      2\  / 3     |
     * |                  |
     *  \ RHPL            |
         \_RHPR___________|
     We name this as if we were looking from the driver station
     The stupid numbering scheme is because of how FRC labeled the sides

     
     */
    
    
   
    public static double[] coral6 = {3.860, 5.122, 124.563  };
    public static double[] coral5 = {5.203, 5.170, 62       };
    public static double[] coral4 = {5.754, 4.055, 0        };
    public static double[] coral3 = {5, 2.9, -61.526        };
    public static double[] coral2 = {3.824, 2.952, -118.276 };
    public static double[] coral1 = {3.225, 3.959, 180      };
    //TODO: Get better name for these
    //TODO: Woahhhh the todo text has blue underlines this is SICK
    public static double[] leftHumanPlayerLeft   = {0.551, 1.382, 57.724        };
    public static double[] leftHumanPlayerRight  = {1.69, 0.591, 57.724        };
    public static double[] rightHumanPlayerLeft  = {0.659, 6.728, 162. -49.399 };
    public static double[] rightHumanPlayerRight = {1.714, 7.483, -49.399     };

  }
}
