// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPDrivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterMeters = 0.06; // 2.3622  in; // 60 mm
  private static final double kTrackWidthMeters = 0.155;

  // The XRP has the left and right motors set tokWheelDiameterInch
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final XRPGyro m_gyro = new XRPGyro();
  private double m_lastGyroAngle = 0;
  private double m_gyroRotations = 0;

  private final DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  private final DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(kTrackWidthMeters);

  private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(kSLeft, kVLeft);
  private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(kSRight, kVRight);

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_gyro.reset();
    m_gyroRotations = 0;
    m_lastGyroAngle = 0;
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch(Exception e) {
      return;
    }

    AutoBuilder.configure(
      m_odometry::getPoseMeters,
      m_odometry::resetPose,
      this::getChassisSpeed,
      this::setChassisSpeed,
      new PPLTVController(0.02, 0.5),
      config,
      ()->false,
      this
    );

    SmartDashboard.putData("XRPDrivetrain", this);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Left Speed", m_leftEncoder::getRate, null);
    builder.addDoubleProperty("Right Speed", m_rightEncoder::getRate, null);
    builder.addDoubleProperty("Left Distance", m_leftEncoder::getDistance, null);
    builder.addDoubleProperty("Right Distance", m_rightEncoder::getDistance, null);
    builder.addDoubleProperty("Battery Voltage", RobotController::getBatteryVoltage, null);
    builder.addDoubleProperty("Gyro Angle", this::getGyroAngle, null);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, false);
    tankDriveFF(kMaxSpeed.times(wheelSpeeds.left), kMaxSpeed.times(wheelSpeeds.right));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()), 
      new DifferentialDriveWheelPositions(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()),
      m_odometry.getPoseMeters()
    );
  }

  public void tankDriveFF(LinearVelocity left, LinearVelocity right) {
    tankDrive(
      left.in(MetersPerSecond) == 0.0 ? Volts.of(0.0) : leftFeedForward.calculate(left), 
      right.in(MetersPerSecond) == 0.0 ? Volts.of(0.0) : rightFeedForward.calculate(right)
    );
  }

  public void tankDrive(Voltage left, Voltage right) {
    m_leftMotor.setVoltage(left);
    m_rightMotor.setVoltage(right);
    //m_diffDrive.tankDrive(left, right);
  }

  public void setChassisSpeed(ChassisSpeeds speed, DriveFeedforwards feedforwards) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speed);
    LinearVelocity leftspeed = MetersPerSecond.of(wheelSpeeds.leftMetersPerSecond);
    LinearVelocity rightspeed = MetersPerSecond.of(wheelSpeeds.rightMetersPerSecond);
    tankDriveFF(leftspeed, rightspeed);
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getRate(), m_rightEncoder.getRate()
      )
    );
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public double getGyroAngle() {
    return m_lastGyroAngle + m_gyroRotations * 360;
  }

  @Override
  public void periodic() {
    double currentGyroAngle = m_gyro.getAngle();
    if(currentGyroAngle < 90 && m_lastGyroAngle > 270) {
      m_gyroRotations++;
    } else if (currentGyroAngle > 270 && m_lastGyroAngle < 90) {
      m_gyroRotations--;
    }
    m_lastGyroAngle = currentGyroAngle;

    m_odometry.update(
      Rotation2d.fromDegrees(getGyroAngle()),
      new DifferentialDriveWheelPositions(m_leftEncoder.getDistance(), m_rightEncoder.getDistance())
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
