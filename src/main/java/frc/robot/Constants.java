// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kRearLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kRearRightDriveMotorPort = 6;
    
        public static final int kFrontLeftTurningMotorPort = 0;
        public static final int kRearLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kRearRightTurningMotorPort = 2;
    
        public static final int[] kFrontLeftTurningEncoderPorts = new int[] {1, 0};
        public static final int[] kRearLeftTurningEncoderPorts = new int[] {5, 4};
        public static final int[] kFrontRightTurningEncoderPorts = new int[] {9, 8};
        public static final int[] kRearRightTurningEncoderPorts = new int[] {7, 6};
    
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kRearLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kRearRightTurningEncoderReversed = false;
        /*
        public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
        public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
        public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
        public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};
    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kRearRightDriveEncoderReversed = false;
        */
        public static final double kDriveP = 1.2539; //15 or 2(latest)
        public static final double kDriveI = 0;//0.01
        public static final double kDriveD = 0;//.1
        //public static final double kDriveF = 0.5; //.2
    
        public static final double kTrackWidth = 0.47;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.50;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        //public static final boolean kGyroReversed = false;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    
        public static final double kMaxSpeedMetersPerSecond = 3;
      }
    
      public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    
        public static final int kDriveEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.10;
        public static final double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kDriveEncoderCPR * 6.67;
            // Assumes the encoders are directly mounted on the wheel shafts
            
        public static final int kTurningEncoderCPR = 7;
        public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) kTurningEncoderCPR * 71 * (40/48);
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            
    
        public static final double kPModuleTurningController = 1;
    
        public static final double kPModuleDriveController = 1;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }

      public static final class LEDConstants {
        
        public static final int kLEDControllerPort = 0;
        public static final double kSparkleonetwo = .37;
        public static final double kSparkletwoone = .39;
        public static final double kGradient = .41;
        public static final double kBeatPerMinute = .43;
        public static final double kEndtoendblendonetwo = .45;
        public static final double kEndtoendblend = .47;
        public static final double kStandardpattern = .49;
        public static final double kTwinkles = .51;
        public static final double kColorWaves = .53;
        public static final double kSinelon = .55;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }


}
