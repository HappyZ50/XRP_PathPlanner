// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turn180 extends Command {
  
  private final XRPDrivetrain m_Drivetrain;
  private final double kp = 0.0001;
  private final double ki = 0.005;
  private final double kd = 0;
  private final PIDController pidController = new PIDController(kp, ki, kd);
  private double targetRotation = 0;
  private double startingRotation = 0;
  private final Timer timer = new Timer();
  private double commandTime = 2.0;

  public Turn180(XRPDrivetrain drivetrain) {
    m_Drivetrain = drivetrain;
    addRequirements(drivetrain);
    SmartDashboard.putData("Turn 180",this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kp", pidController::getP, pidController::setP);
    builder.addDoubleProperty("ki", pidController::getI, pidController::setI);
    builder.addDoubleProperty("kd", pidController::getD, pidController::setD);
    builder.addDoubleProperty("command time", ()->commandTime, (val)->{commandTime=val;});
    builder.addDoubleProperty("start angle", ()->startingRotation, null);
    builder.addDoubleProperty("target angle", ()->targetRotation, null);
    builder.addDoubleProperty("current angle", m_Drivetrain::getGyroAngle, null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    startingRotation = m_Drivetrain.getGyroAngle();
    targetRotation = startingRotation - 180;

    timer.reset();
    timer.start();
  }

  // Called every time the schedgetRequirements()uler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = targetRotation;
    double currentAngle = m_Drivetrain.getGyroAngle();
    m_Drivetrain.arcadeDrive(0, pidController.calculate(currentAngle, targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(commandTime);
  }
}
