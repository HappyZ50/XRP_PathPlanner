package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

public class Characterize extends Command {

    private class MotorCharacteristics {
        public double kS = 0.0;
        public double kV = 0.0;
        public double speed = 0.0;
        public double maxspeed = 0.0;
        public Deque<Double> speeds = new ArrayDeque<>();
        public double speedsum = 0.0;
        public boolean done = false;
    }

    private final XRPDrivetrain m_drivetrain;
    private final Timer timer = new Timer();

    private MotorCharacteristics leftMotorCharacteristics = new MotorCharacteristics();
    private MotorCharacteristics rightMotorCharacteristics = new MotorCharacteristics();

    public Characterize(XRPDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        SmartDashboard.putData("Characterize Command", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("kSLeft", () -> leftMotorCharacteristics.kS, null);
        builder.addDoubleProperty("kVLeft", () -> leftMotorCharacteristics.kV, null);
        builder.addDoubleProperty("kSRight", () -> rightMotorCharacteristics.kS, null);
        builder.addDoubleProperty("kVRight", () -> rightMotorCharacteristics.kV, null);
        builder.addDoubleProperty("left speed", () -> leftMotorCharacteristics.speed, null);
        builder.addDoubleProperty("right speed", () -> rightMotorCharacteristics.speed, null);
        builder.addDoubleProperty("max speed left", () -> leftMotorCharacteristics.maxspeed, null);
        builder.addDoubleProperty("max speed right", () -> rightMotorCharacteristics.maxspeed, null);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        leftMotorCharacteristics = new MotorCharacteristics();
        rightMotorCharacteristics = new MotorCharacteristics();
    }

    private void calculate(MotorCharacteristics motorCharacteristics, double speed, boolean driveLeftMotor) {
        motorCharacteristics.maxspeed = Math.max(speed, motorCharacteristics.maxspeed);
        final Voltage LOW_V = Volts.of(7.0);
        final Voltage HIGH_V = Volts.of(10.0);
        final Voltage Max_V = Volts.of(12.0);
        final int Q_SIZE_MAX = 32;

        Voltage driveVoltage = LOW_V;

        motorCharacteristics.speed = speed;

        if(timer.hasElapsed(25)) {
            motorCharacteristics.done = true;
            timer.reset();
        } else if(timer.hasElapsed(20.0)) {
            if(motorCharacteristics.kS == 0.0) {
                motorCharacteristics.kV = HIGH_V.minus(LOW_V).in(Volts) / (motorCharacteristics.speedsum / motorCharacteristics.speeds.size() - motorCharacteristics.kV);
                motorCharacteristics.kS = HIGH_V.in(Volts) - motorCharacteristics.kV * motorCharacteristics.speedsum / motorCharacteristics.speeds.size();
            }
            driveVoltage = Max_V;
        } else if(timer.hasElapsed(10.0)) {
            if(motorCharacteristics.kV == 0.0) {
                motorCharacteristics.kV = motorCharacteristics.speedsum / motorCharacteristics.speeds.size();
                motorCharacteristics.speeds.clear();
                motorCharacteristics.speedsum = 0.0;
            }
            driveVoltage = HIGH_V;
        } else {
            driveVoltage = LOW_V;
        }

        if(driveLeftMotor) {
            m_drivetrain.tankDrive(driveVoltage, Volts.of(0));
        } else {
            m_drivetrain.tankDrive(Volts.of(0), driveVoltage);
        }

        motorCharacteristics.speedsum += speed;
        motorCharacteristics.speeds.add(speed);
        if(motorCharacteristics.speeds.size() > Q_SIZE_MAX) {
            motorCharacteristics.speedsum -= motorCharacteristics.speeds.pop();
        }
    }

    @Override
    public void execute() {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_drivetrain.getWheelSpeeds();

        if(leftMotorCharacteristics.done) {
            calculate(rightMotorCharacteristics, wheelSpeeds.rightMetersPerSecond, false);
        } else {
            calculate(leftMotorCharacteristics, wheelSpeeds.leftMetersPerSecond, true);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        m_drivetrain.tankDrive(Volts.of(0),Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        return leftMotorCharacteristics.done && rightMotorCharacteristics.done;
    }
}
