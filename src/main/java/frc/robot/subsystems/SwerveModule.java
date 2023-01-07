// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    
    private final WPI_TalonFX m_driveMotor;
    private final WPI_TalonSRX m_turningMotor;
  
    //private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

    
  
    //private final PIDController m_drivePIDController =
    //    new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  
    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  
    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     */
    public SwerveModule(
        
        int driveMotorChannel,
        int turningMotorChannel,
       // int[] driveEncoderPorts,
         int[] turningEncoderPorts,
        /*boolean driveEncoderReversed,*/
        boolean turningEncoderReversed) {

      m_driveMotor = new WPI_TalonFX(driveMotorChannel);
      m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
  
      //this.m_driveEncoder = new Encoder(driveEncoderPorts[4], driveEncoderPorts[7]);
  
      this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);


      TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

        driveTalonFXConfiguration.slot0.kP = DriveConstants.kDriveP;
        driveTalonFXConfiguration.slot0.kI = DriveConstants.kDriveI;
        driveTalonFXConfiguration.slot0.kD = DriveConstants.kDriveD;
        //driveTalonFXConfiguration.slot0.kF = DriveConstants.kDriveF;

        m_driveMotor.configAllSettings(driveTalonFXConfiguration);
        
        m_turningMotor.configFactoryDefault();
       
      // Set the distance per pulse for the drive encoder. We can simply use the
      // distance traveled for one rotation of the wheel divided by the encoder
      // resolution.
      //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
  
      // Set whether drive encoder should be reversed or not
     // m_driveEncoder.setReverseDirection(driveEncoderReversed);
  
      // Set the distance (in this case, angle) per pulse for the turning encoder.
      // This is the the angle through an entire rotation (2 * pi) divided by the
      // encoder resolution.
      m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);
  
      // Set whether turning encoder should be reversed or not
      m_turningEncoder.setReverseDirection(turningEncoderReversed);
  
      // Limit the PID Controller's input range between -pi and pi and set the input
      // to be continuous.
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
  
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderDistancePerPulse * 10, new Rotation2d(m_turningEncoder.get()));
    }
  
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state =
          SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));
  
      // Calculate the drive output from the drive PID controller.
      final double driveOutput = state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
      // m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
  
      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput =
    
          m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());//m_turningEncoder.get(),
  
      // Calculate the turning motor output from the turning PID controller.
      m_driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);
      m_driveMotor.set(driveOutput);
      m_turningMotor.set(turnOutput);
    }
  
    /** Zeros all the SwerveModule encoders. */
    public void resetEncoders() {
      //m_driveEncoder.reset();
      m_turningEncoder.reset();
    }

    public void setTuringPecentOutput(double turnRate){
      m_turningMotor.set(TalonSRXControlMode.PercentOutput, turnRate);
    }

    public int getTurningEncoderCount() {
      int encoderCount = m_turningEncoder.getRaw();
      return encoderCount;
    }



    }
