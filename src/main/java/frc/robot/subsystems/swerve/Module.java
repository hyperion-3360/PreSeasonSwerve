package frc.robot.subsystems.swerve;


import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.ModuleIO.ModuleIOInputs;

public class Module extends SubsystemBase{
   
    private final ModuleIO io;
    private final ModuleIOInputs inputs = new ModuleIOInputs();
    private final int index;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private SysIdRoutine sysIdTranslation;
    private SysIdRoutine sysIdRotation;
    
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public Module(ModuleIO io, int moduleID, 
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
        this.io = io;
        this.index = moduleID;
        this.constants = constants;

            sysIdTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramprate is 1 V/s
                null, // default dynamic voltage is 7 V
                null, // default timeOut is 10s
                // TODO see if signalLogger is better than Logger
                (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
            output -> io.driveSwerve(m_translationCharacterization.withVolts(output).VoltsToApply),
            null,
            this)
            );

             sysIdRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramprate is 1 V/s
                null, // default dynamic voltage is 7 V
                null, // default timeOut is 10s
                // TODO see if signalLogger is better than Logger
                (state) -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
            output -> io.rotateSwerve(m_rotationCharacterization.withRotationalRate(output.in(Volts)).RotationalRate),
            null,
            this)
            );
    }
     }
