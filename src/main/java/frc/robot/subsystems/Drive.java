// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

public class Drive extends SubsystemBase {

    private TalonFXConfiguration configuration = new TalonFXConfiguration();

  // The motors on the left side of the drive.
  private final TalonFX m_leftMotor = new TalonFX(DriveConstants.kLeftMotor1Port);
  private final TalonFX m_left2Motor = new TalonFX(DriveConstants.kLeftMotor2Port);
  

  // The motors on the right side of the drive.
  private final TalonFX m_rightMotor = new TalonFX(DriveConstants.kRightMotor1Port);
  private final TalonFX m_right2Motor = new TalonFX(DriveConstants.kRightMotor2Port);


  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);



  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_leftMotor.setVoltage(volts.in(Volts));
                m_rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftMotor.getVelocity().getValue(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightMotor.getPosition().getValue(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightMotor.getVelocity().getValue(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public Drive() {  
   m_leftMotor.setInverted(true);
      m_rightMotor.setInverted(true);

    m_left2Motor.setControl(new Follower(DriveConstants.kLeftMotor1Port, false));
    m_right2Motor.setControl(new Follower(DriveConstants.kRightMotor1Port, false));

    // m_leftMotor2 = new Follower(DriveConstants.kLeftMotor1Port, false);  
    // m_rightMotor2 = new Follower(DriveConstants.kRightMotor1Port, false);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.


    TalonFXConfigurator cfg = m_leftMotor.getConfigurator();
    cfg.refresh(configuration);

    configuration.Feedback = configuration.Feedback
        .withSensorToMechanismRatio(DriveConstants.gearRatio)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    cfg.apply(configuration);

    TalonFXConfigurator cfg2 = m_leftMotor.getConfigurator();
    cfg2.refresh(configuration);

    configuration.Feedback = configuration.Feedback
        .withSensorToMechanismRatio(DriveConstants.gearRatio)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    cfg2.apply(configuration);
    }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
