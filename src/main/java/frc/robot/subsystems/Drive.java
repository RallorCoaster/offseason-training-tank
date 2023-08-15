package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private CANSparkMax leftLeaderMotor;
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeaderMotor;
    private CANSparkMax rightFollowerMotor;

    public Drive() {
        leftLeaderMotor = new CANSparkMax(4, MotorType.kBrushless);
        leftFollowerMotor = new CANSparkMax(3, MotorType.kBrushless);
        rightLeaderMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightFollowerMotor = new CANSparkMax(2, MotorType.kBrushless);

        leftLeaderMotor.setInverted(false);
        // leftFollowerMotor.setInverted(false);
        rightLeaderMotor.setInverted(true);
        // rightFollowerMotor.setInverted(false);
        leftFollowerMotor.follow(leftLeaderMotor);
        rightFollowerMotor.follow(rightLeaderMotor);

    }

    public Command driveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
        return run(() -> {
            driveArcade(leftAxis.getAsDouble(), rightAxis.getAsDouble());
        });

    }

    public void driveTank(double leftSpeed, double rightSpeed) {
        leftLeaderMotor.setVoltage(leftSpeed * 12.0);
        rightLeaderMotor.setVoltage(rightSpeed * 12.0);
    }

    public void driveArcade(double xSpeed, double zRotation) {
        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        driveTank(leftSpeed, rightSpeed);
    }

    public void driveCurvature(double xSpeed, double zRotation) {

    }

    public void stop() {
        leftLeaderMotor.stopMotor();
        rightLeaderMotor.stopMotor();
    }


}
