package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;


public class Drive extends SubsystemBase {
    private CANSparkMax leftLeaderMotor; 
    private CANSparkMax leftFollowerMotor;
    private CANSparkMax rightLeaderMotor;
    private CANSparkMax rightFollowerMotor;
    private boolean isTankDrive = false;
    private RelativeEncoder leftEncoder; 
    private RelativeEncoder rightEncoder; 
    public static final double wheelRadius = Units.inchesToMeters(6.0) / 2;
    public static final double trackWidth = Units.inchesToMeters(26.0);
    public static final double gearRatio = 10.7;
    private AHRS gyro; 
    private DifferentialDriveOdometry odometry; 
    public DifferentialDriveKinematics kinematics; 
   
    public Drive() {
         leftLeaderMotor = new CANSparkMax (4, MotorType.kBrushless);
         leftFollowerMotor = new CANSparkMax (3, MotorType.kBrushless);
         rightLeaderMotor = new CANSparkMax (1, MotorType.kBrushless);
         rightFollowerMotor = new CANSparkMax (2, MotorType.kBrushless);
         leftEncoder = leftLeaderMotor.getEncoder();
         rightEncoder = rightLeaderMotor.getEncoder();
         gyro = new AHRS();

         odometry = new DifferentialDriveOdometry(getGyroRotation(), 0, 0, new Pose2d());
         kinematics = new DifferentialDriveKinematics(trackWidth);

         leftLeaderMotor.setInverted(false);
         rightLeaderMotor.setInverted(true);
         leftFollowerMotor.follow(leftLeaderMotor, false);
         rightFollowerMotor.follow(rightLeaderMotor, false);
         

         gyro.reset();
         
    }

        @Override
        public void periodic(){
            odometry.update(getGyroRotation(), getLeftDistance(), getRightDistance());
        }
        public Command driveCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
            
            return run(() -> { 
                if (isTankDrive) {
                    DriveTank(leftAxis.getAsDouble(), rightAxis.getAsDouble());
                }
                else {
                    driveArcade(leftAxis.getAsDouble(), rightAxis.getAsDouble());
                }
            
                
            });

            
        }
        public Command switchDriveCommand(){
            return runOnce(() -> {
                isTankDrive = !isTankDrive; 
            });
        }
        public Command moveCertainDistance(){
            return run(() -> {
                DriveTank(.5, .5);
                
            }).withTimeout(2.5);
        }
        
        public Command zeroPoseCommand(){
            return runOnce(() -> resetOdometry(new Pose2d()));
        }

        public void DriveTank(double leftSpeed, double rightSpeed){
            leftLeaderMotor.setVoltage(leftSpeed * 12.0);
            rightLeaderMotor.setVoltage(rightSpeed * 12.0);
        }

        public void driveArcade(double xSpeed, double zRotation){
            double leftSpeed = xSpeed - zRotation;
            double rightSpeed = xSpeed + zRotation;

            DriveTank(leftSpeed, rightSpeed);
        }

        public void stop(){
            leftLeaderMotor.stopMotor();
            rightLeaderMotor.stopMotor();
        }

        public Pose2d getPose() {
            return odometry.getPoseMeters();
        }

        public Rotation2d getRotation(){
            return getPose().getRotation();
        }

        public void resetOdometry(Pose2d pose2d){
            odometry.resetPosition(getGyroRotation(), getLeftDistance(), getRightDistance(), pose2d);
        }

        public double getLeftDistance(){
            return Units.rotationsToRadians(leftEncoder.getPosition() / gearRatio) * wheelRadius;
        }

        public double getRightDistance(){
            return Units.rotationsToRadians(rightEncoder.getPosition() / gearRatio) * wheelRadius;
        }
        
        public void driveVoltage(double leftSpeed, double rightSpeed){
            leftLeaderMotor.setVoltage(leftSpeed);
            rightLeaderMotor.setVoltage(rightSpeed);
        }

        public Rotation2d getGyroRotation(){
            return gyro.getRotation2d();
        }

        public DifferentialDriveWheelSpeeds getWheelSpeeds(){
            return new DifferentialDriveWheelSpeeds(
                Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / gearRatio) * wheelRadius,
                Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / gearRatio) * wheelRadius);
        }
}
