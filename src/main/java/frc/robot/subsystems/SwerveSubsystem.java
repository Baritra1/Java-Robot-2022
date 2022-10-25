//TODO: Fix errors with arrays and PIDCommand constructor
package frc.robot.subsystems;

import frc.robot.RobotMap.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import static java.lang.Math.PI;


public class SwerveSubsystem extends SubsystemBase {

    ChassisSpeeds speeds;

    Translation2d m_frontLeftLocation;
    Translation2d m_backLeftLocation;
    Translation2d m_frontRightLocation;
    Translation2d m_backRightLocation;

    SwerveDriveKinematics m_kinematics;

    SwerveModuleState[] moduleStates;
    private final SwerveSubsystem.SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
    private final SwerveSubsystem.SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
    private final SwerveSubsystem.SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
    private final SwerveSubsystem.SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;

    private final CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
    private final CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
    private final CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
    private final CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;

    private final CANSparkMax LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
    private final CANSparkMax LEFT_BACK_DRIVE_DIRECTION_MOTOR;
    private final CANSparkMax RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
    private final CANSparkMax RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

    private final CANCoder LEFT_FRONT_DRIVE_SPEED_ENCODER;
    private final CANCoder LEFT_BACK_DRIVE_SPEED_ENCODER;
    private final CANCoder RIGHT_FRONT_DRIVE_SPEED_ENCODER;
    private final CANCoder RIGHT_BACK_DRIVE_SPEED_ENCODER;

    public static CANCoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static CANCoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
    public static CANCoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static CANCoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

    SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;
    CANSparkMax[] LEFT_FRONT_SWERVE_MOTORS;
    LEFT_FRONT_SWERVE_MOTORS[0] = LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
    LEFT_FRONT_SWERVE_MOTORS[1] = LEFT_FRONT_DRIVE_SPEED_MOTOR;
    //CANSparkMax[] LEFT_FRONT_SWERVE_MOTORS = [LEFT_FRONT_DRIVE_DIRECTION_MOTOR, LEFT_FRONT_DRIVE_SPEED_MOTOR];
    CANSparkMax[] LEFT_BACK_SWERVE_MOTORS = [LEFT_BACK_DRIVE_DIRECTION_MOTOR, LEFT_BACK_DRIVE_SPEED_MOTOR];
    CANSparkMax[] RIGHT_FRONT_SWERVE_MOTORS = [RIGHT_FRONT_DRIVE_DIRECTION_MOTOR, RIGHT_FRONT_DRIVE_SPEED_MOTOR];
    CANSparkMax[] RIGHT_BACK_SWERVE_MOTORS = [RIGHT_BACK_DRIVE_DIRECTION_MOTOR, RIGHT_BACK_DRIVE_SPEED_MOTOR];
    double wheelP = 0.0;
    double wheelI = 0.0;
    double wheelD = 0.0;
    LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_FRONT_DRIVE_DIRECTION_SCALED, LEFT_FRONT_SWERVE_MOTORS);
    LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_BACK_DRIVE_DIRECTION_SCALED, LEFT_BACK_SWERVE_MOTORS);
    RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_FRONT_DRIVE_DIRECTION_SCALED, RIGHT_FRONT_SWERVE_MOTORS);
    RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_BACK_DRIVE_DIRECTION_SCALED, RIGHT_BACK_SWERVE_MOTORS);


    // public static PIDSourceExtended LEFT_FRONT_DRIVE_DIRECTION_SCALED;
    // public static PIDSourceExtended LEFT_BACK_DRIVE_DIRECTION_SCALED;
    // public static PIDSourceExtended RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
    // public static PIDSourceExtended RIGHT_BACK_DRIVE_DIRECTION_SCALED;    
    public void initDefaultCommand()
    {

    }
    public class SwerveDriveWheel extends PIDCommand
    {
        public SwerveDriveWheel(double kP, double kI, double kD, DoubleSupplier directionSensor, DoubleConsumer directionMotor[0])
        {
            super
            (
                new PIDController(kP,kI,kD),
                directionSensor,
                directionMotor,

            );
            this.directionSensor = m_measurement;
            this.directionMotor = directionMotor[0];
            this.speedMotor = directionMotor[1];
            directionController = new PIDController(kP, kI, kD, directionSensor, directionMotor);
        }

        public void setDirection(double setpoint)
        {
            directionController.reset();
            double currentAngle = directionSensor.get();
            directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));
            directionController.enable();
        }
        //find closest angle between two headings
        private static double closestAngle(double a, double b)
        {
            double dir = modulo(b, 360.0) - modulo(a, 360.0);
            if (Math.abs(dir) > 180.0)
            {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
        }
        public void setSpeed(double speed)
        {
        speedMotor.set(speed);
        }
    }
        /*public SwerveDriveWheel(double kp, double ki, double kd, CANSparkMax directionMotor, CANSparkMax speedMotor, RelativeEncoder directionEncoder, SwerveModuleState mState){


            this.kp = kp;
            this.ki = ki;
            this.kd = kd;

            directionController = new PIDController(this.kp, this.ki, this.kd);

            this.mState = mState;
            this.directionMotor = directionMotor;
            this.speedMotor = speedMotor;
            this.directionEncoder = directionEncoder;
            //What about speed encoder?


        }*/

  /*       public void setSwerveState(SwerveModuleState mState){
            this.mState = mState;
            this.speedMotor.set(this.mState.speedMetersPerSecond / 10);
            this.directionMotor.set(directionController.calculate(directionEncoder.getPosition(), this.mState.angle.getRadians()));
        }

        public double getSpeed(){
            return mState.speedMetersPerSecond;
        }

        public Rotation2d getAngle(){
            return mState.angle;
        }*/


    /*public SwerveSubsystem(){

        m_frontLeftLocation = new Translation2d(Swerve.X_FROM_CENTER, Swerve.Y_FROM_CENTER);
        m_frontRightLocation = new Translation2d(Swerve.X_FROM_CENTER, -Swerve.Y_FROM_CENTER);
        m_backLeftLocation = new Translation2d(-Swerve.X_FROM_CENTER, Swerve.Y_FROM_CENTER);
        m_backRightLocation = new Translation2d(-Swerve.X_FROM_CENTER, -Swerve.Y_FROM_CENTER);

        // Creating my kinematics object using the module locations
        m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
        
        LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(0, MotorType.kBrushless);
        RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(2, MotorType.kBrushless);
        LEFT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(1, MotorType.kBrushless);
        RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new CANSparkMax(3, MotorType.kBrushless);

        LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(4, MotorType.kBrushless);
        RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(6, MotorType.kBrushless);
        LEFT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(5, MotorType.kBrushless);
        RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(7, MotorType.kBrushless);

        LEFT_FRONT_DRIVE_SPEED_ENCODER = LEFT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_FRONT_DRIVE_SPEED_ENCODER = RIGHT_FRONT_DRIVE_SPEED_MOTOR.getEncoder();
        LEFT_BACK_DRIVE_SPEED_ENCODER = LEFT_BACK_DRIVE_SPEED_MOTOR.getEncoder();
        RIGHT_BACK_DRIVE_SPEED_ENCODER = RIGHT_BACK_DRIVE_SPEED_MOTOR.getEncoder();

        LEFT_FRONT_DRIVE_DIRECTION_ENCODER = LEFT_FRONT_DRIVE_DIRECTION_MOTOR.getEncoder();
        RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.getEncoder();
        LEFT_BACK_DRIVE_DIRECTION_ENCODER = LEFT_BACK_DRIVE_DIRECTION_MOTOR.getEncoder();
        RIGHT_BACK_DRIVE_DIRECTION_ENCODER = RIGHT_BACK_DRIVE_DIRECTION_MOTOR.getEncoder();
*/
        /*LEFT_FRONT_DRIVE_DIRECTION_SCALED = new     (LEFT_FRONT_DRIVE_DIRECTION_ENCODER);
        LEFT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(LEFT_BACK_DRIVE_DIRECTION_ENCODER);
        RIGHT_FRONT_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_FRONT_DRIVE_DIRECTION_ENCODER);
        RIGHT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_BACK_DRIVE_DIRECTION_ENCODER);*/
/*
        LEFT_FRONT_DRIVE_WHEEL = new SwerveSubsystem.SwerveDriveWheel(0, 0, 0, 
            LEFT_FRONT_DRIVE_DIRECTION_MOTOR, 
            LEFT_FRONT_DRIVE_SPEED_MOTOR,
            LEFT_FRONT_DRIVE_SPEED_MOTOR.getEncoder(), 
            LEFT_FRONT_DRIVE_DIRECTION_MOTOR.getEncoder(), 
            moduleStates[0]);

        LEFT_BACK_DRIVE_WHEEL = new SwerveSubsystem.SwerveDriveWheel(0, 0, 0, 
            LEFT_BACK_DRIVE_DIRECTION_MOTOR,
            LEFT_BACK_DRIVE_SPEED_MOTOR, 
            LEFT_BACK_DRIVE_SPEED_MOTOR.getEncoder(), 
            LEFT_BACK_DRIVE_DIRECTION_MOTOR.getEncoder(), 
            moduleStates[1]);

        RIGHT_FRONT_DRIVE_WHEEL = new SwerveSubsystem.SwerveDriveWheel(0, 0, 0, 
            RIGHT_FRONT_DRIVE_DIRECTION_MOTOR, 
            RIGHT_FRONT_DRIVE_SPEED_MOTOR.getEncoder(), 
            RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.getEncoder(), 
            moduleStates[2]);

        RIGHT_BACK_DRIVE_WHEEL = new SwerveSubsystem.SwerveDriveWheel(0, 0, 0, 
            RIGHT_BACK_DRIVE_DIRECTION_MOTOR, 
            RIGHT_BACK_DRIVE_SPEED_MOTOR.getEncoder(), 
            RIGHT_BACK_DRIVE_DIRECTION_MOTOR.getEncoder(), 
            moduleStates[3]);
        
    }

    public void setSwerveState(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, Rotation2d robotAngle){
        
        moduleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond,
            vyMetersPerSecond, 
            omegaRadiansPerSecond, 
            robotAngle));

        LEFT_FRONT_DRIVE_WHEEL.setSwerveState(moduleStates[0]);
        RIGHT_FRONT_DRIVE_WHEEL.setSwerveState(moduleStates[1]);
        LEFT_BACK_DRIVE_WHEEL.setSwerveState(moduleStates[2]);
        RIGHT_BACK_DRIVE_WHEEL.setSwerveState(moduleStates[3]);
        
    }
    public void TurnToAngle(Rotation2d angle){
       // LEFT_FRONT_WHEEL
        //.directionMotor(calculate(directionEncoder.getPosition(), this.angle.getRadians()));
    }

}*/
