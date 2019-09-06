package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.frc2019.commands.TeleopDriveCommand
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.wheelRadius
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.asSource
//import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.volts
import sun.security.util.Length
import kotlin.math.absoluteValue

object Drivetrain : FalconWestCoastDrivetrain() {


    // Constants are at the top of each subsystem.
    // These must be private.
    private val kRobotMass = 140.lb
    const val kRobotMomentOfInertia = 10.0
    const val kRobotAngularDrag = 12.0
    val kDriveSensorUnitsPerRotation = 1440.nativeUnits
    private val kDriveNativeUnitModel = SlopeNativeUnitModel(138.inch, 10000.nativeUnits)
    val kDriveWheelRadius = kDriveNativeUnitModel.wheelRadius(kDriveSensorUnitsPerRotation)
    private const val kLeftMasterId = 1
    private const val kLeftSlave1Id = 2
    private const val kRightMasterId = 3
    private const val kRightSlave1Id = 4
    private val kp = 3
    private val volts = 0


    //val kDriveModel =
    /*
    val kDriveModel = DifferentialDrive(
            kRobotMass.value,
            kRobotMomentOfInertia,
            kRobotAngularDrag,
            kDriveWheelRadius.value,
            kDriveTrackWidth.value / 2.0,
            kDriveLeftDCTransmission,
            kDriveRightDCTransmission
    ) */


    private const val kPigeonId = 17

    private const val kBeta = 2.0
    private const val kZeta = 0.7

    // Private member variables
    private val pigeon = PigeonIMU(kPigeonId)

    // Overriden variables

    // Motors
    override val leftMotor: FalconSRX<Meter> = configureGearbox(kLeftMasterId, kLeftSlave1Id, setInverted = false)
    override val rightMotor: FalconSRX<Meter> = configureGearbox(kRightMasterId, kRightSlave1Id, setInverted = true)
    private val leftSlave1: FalconMotor<Meter> = TODO()
    private val rightSlave1: FalconMotor<Meter> = TODO()

    // Motor characterization
    override val leftCharacterization: MotorCharacterization<Meter> = TODO()
    override val rightCharacterization: MotorCharacterization<Meter> = TODO()

    // Gyro
    override val gyro: Source<Rotation2d> = pigeon.asSource()

    // Kinematics
    override val kinematics: DifferentialDriveKinematics = TODO()

    // Odometry
    override val odometry = DifferentialDriveOdometry(kinematics)

    // Trajectory tracker
    override val trajectoryTracker = RamseteTracker(kBeta, kZeta)

    // Constructor
    init {
        defaultCommand = TeleopDriveCommand()
    }

    // Emergency management.
    override fun activateEmergency() {
        listOf(leftMotor, rightMotor).forEach{masterMotor ->
            masterMotor.talonSRX.config_kP(0, 0.0)
            masterMotor.setNeutral()
        }

    }

    override fun recoverFromEmergency() {
        listOf(leftMotor, rightMotor).forEach{masterMotor ->
            masterMotor.talonSRX.config_kP(0, kp.toDouble())
        }
    }


    private fun configureGearbox(masterId: Int, slaveId: Int, setInverted: Boolean): FalconSRX<Meter> {
        val masterMotor = FalconSRX(masterId, kDriveNativeUnitModel)
        val slaveMotor = FalconSRX(slaveId, kDriveNativeUnitModel)

        slaveMotor.follow(masterMotor)
        masterMotor.outputInverted = setInverted
        slaveMotor.outputInverted = setInverted

        masterMotor.feedbackSensor = FeedbackDevice.QuadEncoder

        fun motorSettings(motor: FalconSRX<Meter>){
            motor.talonSRX.configPeakOutputForward(1.0)
            motor.talonSRX.configPeakOutputReverse(-1.0)
            motor.talonSRX.configNominalOutputForward(0.0)
            motor.talonSRX.configNominalOutputReverse(0.0)

            motor.brakeMode = true

            motor.configCurrentLimit(true, FalconSRX.CurrentLimitConfig(
                    80.amp,
                    1.second,
                    38.amp
                )
            )

            motorSettings(masterMotor)
            motorSettings(slaveMotor)

        }
        masterMotor.talonSRX.config_kP(0, kp.toDouble())

        return masterMotor
    }

    override fun periodic() {
        periodicIO.leftVoltage = leftMotor.voltageOutput
        periodicIO.rightVoltage = rightMotor.voltageOutput

        periodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent.amp
        periodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent.amp


    }

    private fun outputs(motor: FalconMotor<Meter>) {
        motor.setDutyCycle()
        motor.setVelocity()
        motor.setVoltage()
    }



}