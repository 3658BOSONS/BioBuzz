package org.firstinspires.ftc.teamcode.pedro.procedures

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.tuning.autotune.Inputs
import com.pedropathing.tuning.autotune.Procedure
import com.pedropathing.tuning.autotune.TuningOpMode
import com.pedropathing.utils.Angle
import com.pedropathing.utils.Utils
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.ArrayDeque
import java.util.Locale
import java.util.function.Function
import kotlin.Boolean
import kotlin.DoubleArray
import kotlin.IllegalStateException
import kotlin.Int
import kotlin.Long
import kotlin.RuntimeException
import kotlin.String
import kotlin.Throws
import kotlin.doubleArrayOf
import kotlin.math.*
import kotlin.math.max
import kotlin.plus
import kotlin.require

class ForesightTuner(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>
) : Procedure("Foresight Tuner", "A procedure for tuning the Foresight Algorithm.") {
	@Throws(InterruptedException::class)
	override fun run() {
		val distanceInput = inputs(
			"Distance",
			"The distance to drive in inches for the Max Achievable Forward and Strafe Identifiers"
		)
		val distance: Inputs.Field<Double?> = distanceInput.d("Distance").withDefault(48.0)
		awaitInputs(distanceInput)
		
		val forwardVelocity: Double = step<Double?>(
			ForwardVelocity(
				localizerFunction,
				drivetrainFunction,
				distance.get()!!
			)
		)!!
		val strafeVelocity: Double =
			step<Double?>(StrafeVelocity(localizerFunction, drivetrainFunction, distance.get()!!))!!
		
		val velocityInput = inputs(
			"Velocity",
			"The velocity to drive to in inches for the Max Achievable Forward and Strafe Deceleration Identifiers"
		)
		val velocity: Inputs.Field<Double?> = velocityInput.d("Velocity").withDefault(30.0)
		awaitInputs(velocityInput)
		
		val forwardDeceleration: Double = step<Double?>(
			ForwardDeceleration(
				localizerFunction,
				drivetrainFunction,
				velocity.get()!!
			)
		)!!
		val strafeDeceleration: Double = step<Double?>(
			StrafeDeceleration(
				localizerFunction,
				drivetrainFunction,
				velocity.get()!!
			)
		)!!
		
		val headingBraking = step<MutableList<Double?>>(
			HeadingBraking(
				localizerFunction,
				drivetrainFunction
			)
		)!!
		var heading: Double = step<Double?>(HeadingTuner(localizerFunction, drivetrainFunction))!!
		
		heading = measured("Heading Tuner", "heading kP", heading, true)
		val headingLinear =
			measured("Heading Braking", "the linear coefficient", headingBraking.get(0)!!, false)
		val headingQuadratic =
			measured("Heading Braking", "the quadratic coefficient", headingBraking.get(1)!!, false)
			
		
		val distanceBrakingInput = inputs(
			"Distance",
			"The distance to drive in inches for the Forward and Strafe Braking Identifiers. Distance must be at least 15 inches for accurate results."
		)
		val distanceBraking: Inputs.Field<Double?> =
			distanceBrakingInput.d("Distance").withDefault(36.0)
		awaitInputs(distanceBrakingInput)
		val safeDistanceBraking = max(distanceBraking.get()!!, 15.0)
		
		val forwardBraking = step<MutableList<Double?>>(
			ForwardBraking(
				localizerFunction,
				drivetrainFunction,
				headingLinear,
				headingQuadratic,
				heading,
				safeDistanceBraking
			)
		)!!
		val strafeBraking = step<MutableList<Double?>>(
			StrafeBraking(
				localizerFunction,
				drivetrainFunction,
				headingLinear,
				headingQuadratic,
				heading,
				safeDistanceBraking
			)
		)!!
		
		val forwardLinear: Double = forwardBraking.get(0)!!
		val forwardQuadratic: Double = forwardBraking.get(1)!!
		val strafeLinear: Double = strafeBraking.get(0)!!
		val strafeQuadratic: Double = strafeBraking.get(1)!!
		
		val forwardTranslational = step<MutableList<Double?>>(
			ForwardTranslational(
				localizerFunction,
				drivetrainFunction
			)
		)!!
		val strafeTranslational = step<MutableList<Double?>>(
			StrafeTranslational(
				localizerFunction,
				drivetrainFunction
			)
		)!!
		
		val forwardTranslationalPrimary: Double = forwardTranslational.get(0)!!
		val forwardTranslationalSecondary: Double = forwardTranslational.get(1)!!
		val coast: Double = forwardTranslational.get(2)!!
		val brake: Double = forwardTranslational.get(3)!!
		
		val strafeTranslationalPrimary: Double = strafeTranslational.get(0)!!
		val strafeTranslationalSecondary: Double = strafeTranslational.get(1)!!
		
		measured("Max Forward Velocity", "the max forward velocity", forwardVelocity, true)
		measured("Max Strafe Velocity", "the max strafe velocity", strafeVelocity, true)
		measured(
			"Forward Deceleration",
			"the natural forward deceleration",
			forwardDeceleration,
			true
		)
		measured("Strafe Deceleration", "the natural strafe deceleration", strafeDeceleration, true)
		measured("Forward Braking", "the linear coefficient", forwardLinear, false)
		measured("Forward Braking", "the quadratic coefficient", forwardQuadratic, false)
		measured("Strafe Braking", "the linear coefficient", strafeLinear, false)
		measured("Strafe Braking", "the quadratic coefficient", strafeQuadratic, false)
		measured("Forward Translational", "the primary kP", forwardTranslationalPrimary, true)
		measured("Forward Translational", "the secondary kP", forwardTranslationalSecondary, true)
		measured("Strafe Translational", "the primary kP", strafeTranslationalPrimary, true)
		measured("Strafe Translational", "the secondary kP", strafeTranslationalSecondary, true)
		measured("Forward Translational", "coast kV", coast, false)
		measured("Forward Translational", "brake kV", brake, false)
		
		result("maxAchievableForwardVelocity", forwardVelocity)
		result("maxAchievableStrafeVelocity", strafeVelocity)
		result("naturalForwardDeceleration", forwardDeceleration)
		result("naturalStrafeDeceleration", strafeDeceleration)
		result("headingBrakingLinearCoefficient", headingLinear)
		result("headingBrakingQuadraticCoefficient", headingQuadratic)
		result("heading kP", heading)
		result("forwardBrakingLinearCoefficient", forwardLinear)
		result("forwardBrakingQuadraticCoefficient", forwardQuadratic)
		result("strafeBrakingLinearCoefficient", strafeLinear)
		result("strafeBrakingQuadraticCoefficient", strafeQuadratic)
		result("forwardTranslational Primary kP", forwardTranslationalPrimary)
		result("forwardTranslational Secondary kP", forwardTranslationalSecondary)
		result("strafeTranslational Primary kP", strafeTranslationalPrimary)
		result("strafeTranslational Secondary kP", strafeTranslationalSecondary)
		result("coast kV", coast)
		result("brake kV", brake)
		
		code(
			Language.JAVA,
			"public static ForesightConfig foresightConfig = new ForesightConfig(\n" +
					"            c -> {\n" +
					"                Controller primaryTranslationalForward = Controller.proportional(" + forwardTranslationalPrimary + ");\n" +
					"                Controller secondaryTranslationalForward = Controller.proportional(" + forwardTranslationalSecondary + ");\n" +
					"                Controller primaryTranslationalLateral = Controller.proportional(" + strafeTranslationalPrimary + ");\n" +
					"                Controller secondaryTranslationalLateral = Controller.proportional(" + strafeTranslationalSecondary + ");\n" +
					"\n" +
					"                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));\n" +
					"                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));\n" +
					"\n" +
					"                c.coast.set(Controller.proportionalFeedforward(" + coast + "));\n" +
					"                c.brake.set(Controller.proportionalFeedforward(" + brake + "));\n" +
					"\n" +
					"                c.headingFeedback.set(Controller.proportional(" + heading + "));\n" +
					"                c.headingBrakeCoefficients.set(Vector2D.cartesian(" + headingLinear + ", " + headingQuadratic + "));\n" +
					"\n" +
					"                c.linearBrakeCoefficients.set(Matrix.diag(" + forwardLinear + ", " + strafeLinear + "));\n" +
					"                c.quadraticBrakeCoefficients.set(Matrix.diag(" + forwardQuadratic + ", " + strafeQuadratic + "));\n" +
					"\n" +
					"                c.maxAchievableForwardVelocity.set(" + forwardVelocity + ");\n" +
					"                c.maxAchievableStrafeVelocity.set(" + strafeVelocity + ");\n" +
					"                c.naturalForwardDeceleration.set(" + forwardDeceleration + ");\n" +
					"                c.naturalStrafeDeceleration.set(" + strafeDeceleration + ");\n" +
					"            }\n" +
					"    );"
		)
	}
	
	/**
	 * Runs one step. If it fails, the tuning stops with the step's name and reason instead of the
	 * exception ending the procedure without a message.
	 */
	@Throws(InterruptedException::class)
	private fun <T> step(opMode: TuningOpMode<T?>): T? {
		try {
			return runOpMode<T?>(opMode)
		} catch (e: RuntimeException) {
			abort(opMode.name + " failed: " + e.message)
			throw e // not reached: abort() throws
		}
	}
	
	/**
	 * A measured value that later steps drive with, or that goes into the generated config. Stops the
	 * tuning if it is NaN or infinite, or not positive where it must be. A NaN heading kP, for example,
	 * made every power the braking steps sent NaN; CachedMotor ignores NaN powers, so the motors kept
	 * their last power and the robot drove off.
	 */
	@Throws(InterruptedException::class)
	private fun measured(step: String?, name: String?, value: Double, positive: Boolean): Double {
		if (value.isInfinite() || (positive && value <= 0)) {
			abort(step + " measured " + name + " = " + value + ", which can't be used. Run the tuner again.")
		}
		return value
	}
}

internal class ForwardVelocity(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var distance: Double
) : TuningOpMode<Double?>(
	"Max Forward Velocity",
	"A tuner for finding the maximum achievable forward velocity. This will drive forward for " + distance + " inches and then likely drift past that position.",
	false
) {
	private val velocities = ArrayDeque<Double?>()
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		var end = false
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		val power = DrivePowers(1.0, 0.0, 0.0)
		
		var i = 0
		while (i < RECORD_NUMBER) {
			velocities.add(0.0)
			i++
		}
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		while (!end) {
			localizer.update()
			if (abs(localizer.pose().x()) > distance) {
				end = true
				drivetrain.stop()
			} else {
				drivetrain.drive(power, true)
				val currentVelocity = abs(localizer.twist().toVector2D().x())
				velocities.addLast(currentVelocity)
				velocities.removeFirst()
			}
		}
		
		drivetrain.stop()
		var average = 0.0
		for (velocity in velocities) {
			average += velocity!!
		}
		average /= velocities.size.toDouble()
		return average
	}
	
	companion object {
		var RECORD_NUMBER: Double = 10.0
	}
}

internal class StrafeVelocity(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var distance: Double
) : TuningOpMode<Double?>(
	"Max Strafe Velocity",
	"A tuner for finding the maximum achievable strafe velocity. This will drive left for " + distance + " inches and then likely drift past that position.",
	false
) {
	private val velocities = ArrayDeque<Double?>()
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		var end = false
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		val power = DrivePowers(0.0, 1.0, 0.0)
		
		var i = 0
		while (i < RECORD_NUMBER) {
			velocities.add(0.0)
			i++
		}
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		while (!end) {
			localizer.update()
			if (abs(localizer.pose().y()) > distance) {
				end = true
				drivetrain.stop()
			} else {
				drivetrain.drive(power, false)
				val currentVelocity = abs(localizer.twist().toVector2D().y())
				velocities.addLast(currentVelocity)
				velocities.removeFirst()
			}
		}
		
		drivetrain.stop()
		var average = 0.0
		for (velocity in velocities) {
			average += velocity!!
		}
		average /= velocities.size.toDouble()
		return average
	}
	
	companion object {
		var RECORD_NUMBER: Double = 10.0
	}
}

internal class ForwardDeceleration(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var velocity: Double
) : TuningOpMode<Double?>(
	"Forward Deceleration",
	"A tuner for finding the deceleration of the robot when moving forward. This will move forward until it reaches " + velocity + " inches per second.",
	false
) {
	private val accelerations = ArrayList<Double?>()
	
	private var previousVelocity = 0.0
	private var previousTimeNano: Long = 0
	private var stopping = false
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		accelerations.clear()
		previousVelocity = 0.0
		previousTimeNano = 0
		stopping = false
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		val power = DrivePowers(1.0, 0.0, 0.0)
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		drivetrain.drive(power, false)
		
		while (!stopping) {
			localizer.update()
			val currentVelocity = localizer.twist().toVector2D().x()
			if (abs(currentVelocity) > velocity) {
				previousVelocity = currentVelocity
				previousTimeNano = System.nanoTime()
				
				stopping = true
				drivetrain.stop(false)
			} else {
				drivetrain.drive(power, false)
			}
		}
		
		var end = false
		
		while (!end) {
			localizer.update()
			drivetrain.stop(false)
			val currentVelocity = localizer.twist().toVector2D().x()
			val currentTimeNano = System.nanoTime()
			val dt = (currentTimeNano - previousTimeNano) / 1e9
			
			if (dt > 0) {
				val acceleration = (currentVelocity - previousVelocity) / dt
				accelerations.add(acceleration)
			}
			
			previousVelocity = currentVelocity
			previousTimeNano = currentTimeNano
			
			if (abs(currentVelocity) <= 1) {
				end = true
			}
		}
		
		drivetrain.stop(false)
		
		var average = 0.0
		
		for (acceleration in accelerations) {
			average += acceleration!!
		}
		
		if (accelerations.isEmpty()) {
			return 0.0
		}
		
		average /= accelerations.size.toDouble()
		
		return abs(average)
	}
}

internal class StrafeDeceleration(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	var velocity: Double
) : TuningOpMode<Double?>(
	"Strafe Deceleration",
	"A tuner for finding the deceleration of the robot when moving laterally. This will drive left until it reaches " + velocity + " inches per second.",
	false
) {
	private val accelerations = ArrayList<Double?>()
	
	private var previousVelocity = 0.0
	private var previousTimeNano: Long = 0
	private var stopping = false
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		accelerations.clear()
		previousVelocity = 0.0
		previousTimeNano = 0
		stopping = false
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		val power = DrivePowers(0.0, 1.0, 0.0)
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		
		drivetrain.drive(power, false)
		
		while (!stopping) {
			localizer.update()
			val currentVelocity = localizer.twist().toVector2D().y()
			if (abs(currentVelocity) > velocity) {
				previousVelocity = currentVelocity
				previousTimeNano = System.nanoTime()
				
				stopping = true
				drivetrain.stop(false)
			} else {
				drivetrain.drive(power, false)
			}
		}
		
		var end = false
		
		while (!end) {
			drivetrain.stop(false)
			localizer.update()
			val currentVelocity = localizer.twist().toVector2D().y()
			val currentTimeNano = System.nanoTime()
			val dt = (currentTimeNano - previousTimeNano) / 1e9
			
			if (dt > 0) {
				val acceleration = (currentVelocity - previousVelocity) / dt
				accelerations.add(acceleration)
			}
			
			previousVelocity = currentVelocity
			previousTimeNano = currentTimeNano
			
			if (abs(currentVelocity) <= 1) {
				end = true
			}
		}
		
		drivetrain.stop(false)
		
		var average = 0.0
		
		for (acceleration in accelerations) {
			average += acceleration!!
		}
		
		if (accelerations.isEmpty()) {
			return 0.0
		}
		
		average /= accelerations.size.toDouble()
		
		return abs(average)
	}
}

internal class HeadingBraking //    private VoltageSensor voltageSensor;
	(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>
) : TuningOpMode<MutableList<Double?>?>(
	"Heading Braking",
	"A tuner for finding the Heading Braking Coefficients. The robot will turn back at forth at various speed levels.",
	false
) {
	private val timer = ElapsedTime()
	
	private val velocityToBrakingDistance: MutableList<DoubleArray?> = ArrayList<DoubleArray?>()
	private var state = State.DRIVE
	private var iteration = 0
	private var direction = 0
	private var power = 0.0
	
	private var startHeading = 0.0
	private var measuredVelocity = 0.0
	private var totalHeading = 0.0
	private var previousHeading = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		var coefficients = mutableListOf<Double?>()
		
		POWERS = biasedGradient(trials, maxPower, minPower, bias)
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		
		while (state != State.DONE && !isStopRequested()) {
			localizer.update()
			val currentHeading = localizer.pose().heading()
			totalHeading += Angle.normalizeSigned(currentHeading - previousHeading)
			previousHeading = currentHeading
			
			direction = if (iteration % 2 == 0) 1 else -1
			if (iteration < POWERS.size) {
				power = POWERS[iteration]
			}
			
			when (state) {
				State.DRIVE -> {
					if (timer.seconds() > 2) {
						startHeading = totalHeading
						measuredVelocity = abs(localizer.velocity().omega)
						
						drivetrain.drive(DrivePowers(0.0, 0.0, -brakingPower * direction), false)
						state = State.BRAKE
						timer.reset()
						continue // Use continue instead of break to proceed to the next loop iteration
					}
					drivetrain.drive(DrivePowers(0.0, 0.0, power * direction), false)
				}
				
				State.BRAKE -> {
					if (abs(localizer.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
						drivetrain.drive(DrivePowers(0.0, 0.0, -brakingPower * direction), false)
						continue // Keep looping in BRAKE state until motion stops or timeout is reached
					}
					
					val endHeading = totalHeading
					val brakingDistance = abs(endHeading - startHeading)
					
					velocityToBrakingDistance.add(doubleArrayOf(measuredVelocity, brakingDistance))
					
					iteration++
					
					if (iteration >= POWERS.size) {
						drivetrain.stop()
						
						val c = Utils.quadraticFit(velocityToBrakingDistance)
						coefficients = mutableListOf<Double?>(c[0], c[1])
						
						state = State.DONE
					} else {
						timer.reset()
						state = State.DRIVE
					}
				}
				
				State.DONE -> {}
			}
		}
		
		
		return coefficients
	}
	
	private enum class State {
		DRIVE,
		BRAKE,
		DONE
	}
	
	companion object {
		private lateinit var POWERS: DoubleArray
		var MAX_BRAKE_TIME: Double =
			3.0 //seconds, the robot shouldn't take longer than this to brake
		
		var trials: Int = 12
		var maxPower: Double = 1.0
		var minPower: Double = 0.2
		var bias: Double = 1.5 // how much it favors doing trials with higher powers
		var brakingPower: Double = 0.001
		
		private fun biasedGradient(
			count: Int,
			max: Double,
			min: Double,
			bias: Double
		): DoubleArray {
			if (count < 2) return doubleArrayOf(max)
			
			val values = DoubleArray(count)
			
			for (i in 0..<count) {
				val t = i.toDouble() / (count - 1)
				
				val curved = 1 - t.pow(bias)
				
				values[i] = min + curved * (max - min)
			}
			
			return values
		}
	}
}

internal class HeadingTuner(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>
) : TuningOpMode<Double?>(
	"Heading Tuner",
	"A tuner for finding the Heading Tuning Coefficients using system identification. This will spin the robot in place for a couple seconds.",
	false
) {
	private var tau = 0.0
	private var K = 0.0
	private var kV = 0.0
	private var kA = 0.0
	private var vMax = 0.0
	private val times: MutableList<Double?> = ArrayList<Double?>()
	private val velocities: MutableList<Double?> = ArrayList<Double?>()
	private val timer = ElapsedTime()
	private var done = false
	private var lastTime = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): Double {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		times.clear()
		velocities.clear()
		done = false
		vMax = 0.0
		lastTime = 0.0
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		lastTime = timer.seconds()
		drivetrain.drive(DrivePowers(0.0, 0.0, POWER), false)
		
		while (!done && !isStopRequested()) {
			val now = timer.seconds()
			var dt = now - lastTime
			if (dt <= 0) dt = 1e-6
			lastTime = now
			
			localizer.update()
			
			if (!done) {
				times.add(timer.seconds())
				
				val turnVel = abs(localizer.velocity().omega)
				vMax = max(vMax, turnVel / POWER)
				
				velocities.add(turnVel)
				
				if (timer.seconds() >= RUNTIME) {
					done = true
					systemIdentification()
					drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), false)
				} else {
					drivetrain.drive(DrivePowers(0.0, 0.0, POWER), false)
				}
			}
		}
		
		drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), true)
		return calculatekP(ALPHA)
	}
	
	private fun calculatekP(alpha: Double): Double {
		kV = 1 / K
		kA = tau / K
		return tau * alpha * alpha / K
	}
	
	private fun systemIdentification() {
		val N = times.size
		require(N >= 4) { "Failed calibration." }
		
		val start = max(0, N - SAMPLES)
		val samples = (N - start).toDouble()
		var sum = 0.0
		for (i in start..<N) sum += velocities.get(i)!!
		val A = sum / samples
		this.K = A / POWER
		
		val y: MutableList<Double?> = ArrayList<Double?>()
		val x: MutableList<Double?> = ArrayList<Double?>()
		for (i in 0..<N) {
			val vel: Double = velocities.get(i)!! / POWER
			if (vel > 0.8 * K) continue
			if (vel < 0.1 * K) continue
			y.add(ln(K - vel))
			x.add(times.get(i))
		}
		val linReg = Utils.linearFit(
			x.toTypedArray<Double?>(),
			y.toTypedArray<Double?>()
		)
		// With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
		require(!(x.size < 2 || linReg[1].isInfinite() || linReg[1] >= 0)) {
			("Failed calibration: only " + x.size
					+ " samples while the robot sped up.")
		}
		this.tau = -1.0 / linReg[1]
	}
	
	companion object {
		private const val POWER = 0.4
		private const val RUNTIME = 1.2
		private const val SAMPLES = 15
		var ALPHA: Double = 18.25
	}
}

internal class ForwardBraking(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	private val headingLinear: Double,
	private val headingQuadratic: Double,
	private val headingKP: Double,
	var distance: Double
) : TuningOpMode<MutableList<Double?>?>(
	"Forward Braking",
	"A tuner for finding the Forward Braking Coefficients by driving forward and backward at various speeds. Please ensure that you have plenty of room at least " + distance + " inches ahead of the robot, but also tile space behind and laterally around the robot.",
	false
) {
	private lateinit var POWERS: DoubleArray
	var MAX_BRAKE_TIME: Double = 7.0
	var trials: Int = 5
	var maxPower: Double = 0.7
	var minPower: Double = 0.3
	var bias: Double = 1.5
	var brakingPower: Double = 0.001
	var IDLE_SECONDS: Double = 1.0
	
	/** Stops the robot if it gets this many inches past the test's ends.  */
	var OVERRUN: Double = 24.0
	
	/** Stops the robot if it drifts this many inches across the test's direction.  */
	var DRIFT: Double = 48.0
	
	private val timer = ElapsedTime()
	private val velocityToBrakingDistance: MutableList<DoubleArray?> = ArrayList<DoubleArray?>()
	private var state = State.DRIVE
	private var iteration = 0
	private var direction = 0
	private var power = 0.0
	private var startPosition: Vector2D? = null
	private var measuredVelocity = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		POWERS = biasedGradient(trials, maxPower, minPower, bias)
		
		var coefficients = mutableListOf<Double?>()
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		
		drivetrain.drive(DrivePowers(maxPower, 0.0, 0.0), false)
		
		while (state != State.DONE && !isStopRequested()) {
			localizer.update()
			if (abs(localizer.pose().x()) > distance + OVERRUN || abs(
					localizer.pose().y()
				) > DRIFT
			) {
				drivetrain.stop()
				throw IllegalStateException(
					String.format(
						Locale.US,
						"stopped the robot at (%.0f, %.0f): more than %.0f in past the test's ends or %.0f in across them",
						localizer.pose().x(), localizer.pose().y(), OVERRUN, DRIFT
					)
				)
			}
			direction = if (iteration % 2 == 0) 1 else -1
			if (iteration < POWERS.size) {
				power = POWERS[iteration]
			}
			
			when (state) {
				State.DRIVE -> {
					if ((direction > 0 && abs(localizer.pose().x()) >= distance) || (direction < 0 && abs(localizer.pose().x()) <= 12)) {
						startPosition = localizer.pose().toVector2D()
						measuredVelocity = localizer.velocity().toVector2D().magnitude()
						
						brake(drivetrain, localizer)
						state = State.BRAKE
						timer.reset()
						continue // Change from break to continue
					}
					drive(drivetrain, localizer)
				}
				
				State.BRAKE -> {
					if (localizer.velocity().toVector2D().x() * direction > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
						brake(drivetrain, localizer)
						continue // Change from break to continue
					}
					
					collectTrialData(localizer, drivetrain)
				}
				
				State.WAIT -> {
					drivetrain.stop()
					if (timer.seconds() > IDLE_SECONDS) state = State.DRIVE
				}
				
				State.DONE -> {}
			}
		}
		
		if (state == State.DONE) {
			val c = Utils.quadraticFit(velocityToBrakingDistance)
			coefficients = mutableListOf<Double?>(c[0], c[1])
		}
		
		return coefficients
	}
	
	private fun getHeadingPower(localizer: Localizer): Double {
		val angularVel = localizer.velocity().omega
		val brakeDist = headingLinear * angularVel +
				headingQuadratic * angularVel * angularVel * sign(angularVel)
		val headingError = Angle.normalizeSigned(-localizer.pose().heading())
		val error = headingError - brakeDist
		return Utils.clamp(headingKP * error, -0.3, 1.0) / 2
	}
	
	
	private fun drive(drivetrain: Drivetrain, localizer: Localizer) {
		drivetrain.drive(DrivePowers(power * direction, 0.0, getHeadingPower(localizer)), false)
	}
	
	private fun brake(drivetrain: Drivetrain, localizer: Localizer) {
		val headingPower = getHeadingPower(localizer)
		var brake = -brakingPower * direction
		val minBrake = abs(headingPower) + 0.001
		
		if (direction > 0) {
			brake = min(brake, -minBrake)
		} else {
			brake = max(brake, minBrake)
		}
		
		drivetrain.drive(DrivePowers(brake, 0.0, headingPower), false)
	}
	
	private fun collectTrialData(localizer: Localizer, drivetrain: Drivetrain) {
		val endPosition = localizer.pose().toVector2D()
		val brakingDistance = endPosition.minus(startPosition).magnitude()
		
		velocityToBrakingDistance.add(doubleArrayOf(measuredVelocity, brakingDistance))
		
		iteration++
		
		if (iteration >= POWERS.size) {
			drivetrain.stop()
			state = State.DONE
		} else {
			state = State.WAIT
			timer.reset()
		}
	}
	
	private enum class State {
		DRIVE,
		BRAKE,
		WAIT,
		DONE
	}
	
	companion object {
		private fun biasedGradient(
			count: Int,
			max: Double,
			min: Double,
			bias: Double
		): DoubleArray {
			if (count < 2) return doubleArrayOf(max)
			val values = DoubleArray(count)
			for (i in 0..<count) {
				val t = i.toDouble() / (count - 1)
				val curved = 1 - t.pow(bias)
				values[i] = min + curved * (max - min)
			}
			return values
		}
	}
}

internal class StrafeBraking(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>,
	private val headingLinear: Double,
	private val headingQuadratic: Double,
	private val headingKP: Double,
	var distance: Double
) : TuningOpMode<MutableList<Double?>?>(
	"Strafe Braking",
	"A tuner for finding the Strafe Braking Coefficients by strafing left and right at various speeds. Please ensure that you have plenty of room at least " + distance + " inches to the left and right of the robot and space in front and behind the robot.",
	false
) {
	private lateinit var POWERS: DoubleArray
	var MAX_BRAKE_TIME: Double = 7.0
	var trials: Int = 5
	var maxPower: Double = 1.0
	var minPower: Double = 0.3
	var bias: Double = 1.5
	var brakingPower: Double = 0.001
	var IDLE_SECONDS: Double = 1.0
	
	/** Stops the robot if it gets this many inches past the test's ends.  */
	var OVERRUN: Double = 24.0
	
	/** Stops the robot if it drifts this many inches across the test's direction.  */
	var DRIFT: Double = 48.0
	
	private val timer = ElapsedTime()
	private val velocityToBrakingDistance: MutableList<DoubleArray?> = ArrayList<DoubleArray?>()
	private var state = State.DRIVE
	private var iteration = 0
	private var direction = 0
	private var power = 0.0
	private var startPosition: Vector2D? = null
	private var measuredVelocity = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		POWERS = biasedGradient(trials, maxPower, minPower, bias)
		
		var coefficients = mutableListOf<Double?>()
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		
		drivetrain.drive(DrivePowers(0.0, maxPower, 0.0), false)
		
		while (state != State.DONE && !isStopRequested()) {
			localizer.update()
			if (abs(localizer.pose().y()) > distance + OVERRUN || abs(
					localizer.pose().x()
				) > DRIFT
			) {
				drivetrain.stop()
				throw IllegalStateException(
					String.format(
						Locale.US,
						"stopped the robot at (%.0f, %.0f): more than %.0f in past the test's ends or %.0f in across them",
						localizer.pose().x(), localizer.pose().y(), OVERRUN, DRIFT
					)
				)
			}
			direction = if (iteration % 2 == 0) 1 else -1
			if (iteration < POWERS.size) {
				power = POWERS[iteration]
			}
			
			when (state) {
				State.DRIVE -> {
					if ((direction > 0 && abs(localizer.pose().y()) > distance) || (direction < 0 && abs(localizer.pose().y()) <= 6)) {
						startPosition = localizer.pose().toVector2D()
						measuredVelocity = localizer.velocity().toVector2D().magnitude()
						
						brake(drivetrain, localizer)
						state = State.BRAKE
						timer.reset()
						continue // Change from break to continue
					}
					drive(drivetrain, localizer)
				}
				
				State.BRAKE -> {
					if (localizer.velocity().toVector2D().y() * direction > 0.25 && timer.seconds() < MAX_BRAKE_TIME) {
						brake(drivetrain, localizer)
						continue // Change from break to continue
					}
					
					collectTrialData(localizer, drivetrain)
				}
				
				State.WAIT -> {
					drivetrain.stop()
					if (timer.seconds() > IDLE_SECONDS) state = State.DRIVE
				}
				
				State.DONE -> {}
			}
		}
		
		if (state == State.DONE) {
			val c = Utils.quadraticFit(velocityToBrakingDistance)
			coefficients = mutableListOf<Double?>(c[0], c[1])
		}
		
		return coefficients
	}
	
	private fun getHeadingPower(localizer: Localizer): Double {
		val angularVel = localizer.velocity().omega
		val brakeDist = headingLinear * angularVel +
				headingQuadratic * angularVel * angularVel * sign(angularVel)
		val headingError = Angle.normalizeSigned(-localizer.pose().heading())
		val error = headingError - brakeDist
		return Utils.clamp(headingKP * error, -0.3, 1.0) / 2
	}
	
	private fun drive(drivetrain: Drivetrain, localizer: Localizer) {
		drivetrain.drive(DrivePowers(0.0, power * direction, getHeadingPower(localizer)), false)
	}
	
	private fun brake(drivetrain: Drivetrain, localizer: Localizer) {
		val headingPower = getHeadingPower(localizer)
		var brake = -brakingPower * direction
		val minBrake = abs(headingPower) + 0.001
		
		if (direction > 0) {
			brake = min(brake, -minBrake)
		} else {
			brake = max(brake, minBrake)
		}
		
		drivetrain.drive(DrivePowers(0.0, brake, headingPower), false)
	}
	
	private fun collectTrialData(localizer: Localizer, drivetrain: Drivetrain) {
		val endPosition = localizer.pose().toVector2D()
		val brakingDistance = endPosition.minus(startPosition).magnitude()
		
		velocityToBrakingDistance.add(doubleArrayOf(measuredVelocity, brakingDistance))
		
		iteration++
		
		if (iteration >= POWERS.size) {
			drivetrain.stop()
			state = State.DONE
		} else {
			state = State.WAIT
			timer.reset()
		}
	}
	
	private enum class State {
		DRIVE,
		BRAKE,
		WAIT,
		DONE
	}
	
	companion object {
		private fun biasedGradient(
			count: Int,
			max: Double,
			min: Double,
			bias: Double
		): DoubleArray {
			if (count < 2) return doubleArrayOf(max)
			val values = DoubleArray(count)
			for (i in 0..<count) {
				val t = i.toDouble() / (count - 1)
				val curved = 1 - t.pow(bias)
				values[i] = min + curved * (max - min)
			}
			return values
		}
	}
}

internal class ForwardTranslational(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>
) : TuningOpMode<MutableList<Double?>?>(
	"Forward Translational",
	"A tuner for finding the Forward Translational kP coefficients using system identification. This will move around 12-24 inches in front of the robot and then stop.",
	false
) {
	private val VEL_AGGRESSIVENESS = 0.85
	private val POWER = 0.4
	private val RUNTIME = 1.2
	private val SAMPLES = 15
	
	private var tau = 0.0
	private var K = 0.0
	private var kV = 0.0
	private var kA = 0.0
	private var vMax = 0.0
	private val times: MutableList<Double?> = ArrayList<Double?>()
	private val velocities: MutableList<Double?> = ArrayList<Double?>()
	private val timer = ElapsedTime()
	private var done = false
	private var lastTime = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		times.clear()
		velocities.clear()
		done = false
		vMax = 0.0
		lastTime = 0.0
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		lastTime = timer.seconds()
		drivetrain.drive(DrivePowers(POWER, 0.0, 0.0), false)
		
		while (!done && !isStopRequested()) {
			val now = timer.seconds()
			var dt = now - lastTime
			if (dt <= 0) dt = 1e-6
			lastTime = now
			
			localizer.update()
			
			if (!done) {
				times.add(timer.seconds())
				
				val forwardVelocity = abs(localizer.twist().toVector2D().x())
				vMax = max(vMax, forwardVelocity / POWER)
				
				velocities.add(forwardVelocity)
				
				if (timer.seconds() >= RUNTIME) {
					done = true
					systemIdentification()
					drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), false)
				} else {
					drivetrain.drive(DrivePowers(POWER, 0.0, 0.0), false)
				}
			}
		}
		
		drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), true)
		
		val kP_large = calculatekP(ALPHA_LARGE)
		val kP_small = calculatekP(ALPHA_SMALL)
		
		//  kP_large, kP_small, coast kV, and brake kV (scaled by aggressiveness factor)
		return mutableListOf<Double?>(kP_large, kP_small, kV, kV * VEL_AGGRESSIVENESS)
	}
	
	private fun calculatekP(alpha: Double): Double {
		kV = 1 / K
		kA = tau / K
		return tau * alpha * alpha / K
	}
	
	private fun systemIdentification() {
		val N = times.size
		require(N >= 4) { "Failed calibration." }
		
		val start = max(0, N - SAMPLES)
		val samples = (N - start).toDouble()
		var sum = 0.0
		for (i in start..<N) sum += velocities.get(i)!!
		val A = sum / samples
		this.K = A / POWER
		
		val y: MutableList<Double?> = ArrayList<Double?>()
		val x: MutableList<Double?> = ArrayList<Double?>()
		for (i in 0..<N) {
			val vel = velocities.get(i)!! / POWER
			if (vel > 0.8 * K) continue
			if (vel < 0.1 * K) continue
			y.add(ln(K - vel))
			x.add(times.get(i))
		}
		val linReg = Utils.linearFit(
			x.toTypedArray<Double?>(),
			y.toTypedArray<Double?>()
		)
		// With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
		require(!(x.size < 2 || linReg[1].isInfinite() || linReg[1] >= 0)) {
			("Failed calibration: only " + x.size
					+ " samples while the robot sped up.")
		}
		this.tau = -1.0 / linReg[1]
	}
	
	companion object {
		var ALPHA_LARGE: Double = 10.2
		var ALPHA_SMALL: Double = 6.2
	}
}

internal class StrafeTranslational(
	var localizerFunction: Function<HardwareMap?, Localizer>,
	var drivetrainFunction: Function<HardwareMap?, Drivetrain>
) : TuningOpMode<MutableList<Double?>?>(
	"Strafe Translational",
	"A tuner for finding the Strafe Translational kP coefficients using system identification. This will move around 12-24 inches to the left and right of the robot and then stop.",
	false
) {
	private val POWER = 0.4
	private val RUNTIME = 1.2
	private val SAMPLES = 15
	
	private var tau = 0.0
	private var K = 0.0
	private var kV = 0.0
	private var kA = 0.0
	private var vMax = 0.0
	private val times: MutableList<Double?> = ArrayList<Double?>()
	private val velocities: MutableList<Double?> = ArrayList<Double?>()
	private val timer = ElapsedTime()
	private var done = false
	private var lastTime = 0.0
	
	@Throws(InterruptedException::class)
	override fun runTuningOpMode(): MutableList<Double?> {
		val localizer = localizerFunction.apply(hardwareMap)
		val drivetrain = drivetrainFunction.apply(hardwareMap)
		
		localizer.setPose(Pose.zero())
		localizer.update()
		
		times.clear()
		velocities.clear()
		done = false
		vMax = 0.0
		lastTime = 0.0
		
		Thread.sleep(1000)
		waitForStart()
		localizer.setPose(Pose.zero())
		localizer.update()
		timer.reset()
		lastTime = timer.seconds()
		drivetrain.drive(DrivePowers(0.0, POWER, 0.0), false)
		
		while (!done && !isStopRequested()) {
			val now = timer.seconds()
			var dt = now - lastTime
			if (dt <= 0) dt = 1e-6
			lastTime = now
			
			localizer.update()
			
			if (!done) {
				times.add(timer.seconds())
				
				val lateralVelocity = abs(localizer.twist().toVector2D().y())
				vMax = max(vMax, lateralVelocity / POWER)
				
				velocities.add(lateralVelocity)
				
				if (timer.seconds() >= RUNTIME) {
					done = true
					systemIdentification()
					drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), false)
				} else {
					drivetrain.drive(DrivePowers(0.0, POWER, 0.0), false)
				}
			}
		}
		
		drivetrain.drive(DrivePowers(0.0, 0.0, 0.0), true)
		
		val kP_large = calculatekP(ALPHA_LARGE)
		val kP_small = calculatekP(ALPHA_SMALL)
		
		return mutableListOf<Double?>(kP_large, kP_small)
	}
	
	private fun calculatekP(alpha: Double): Double {
		kV = 1 / K
		kA = tau / K
		return tau * alpha * alpha / K
	}
	
	private fun systemIdentification() {
		val N = times.size
		require(N >= 4) { "Failed calibration." }
		
		val start = max(0, N - SAMPLES)
		val samples = (N - start).toDouble()
		var sum = 0.0
		for (i in start..<N) sum += velocities.get(i)!!
		val A = sum / samples
		this.K = A / POWER
		
		val y: MutableList<Double?> = ArrayList<Double?>()
		val x: MutableList<Double?> = ArrayList<Double?>()
		for (i in 0..<N) {
			val vel = velocities.get(i)!! / POWER
			if (vel > 0.8 * K) continue
			if (vel < 0.1 * K) continue
			y.add(ln(K - vel))
			x.add(times.get(i))
		}
		val linReg = Utils.linearFit(
			x.toTypedArray<Double?>(),
			y.toTypedArray<Double?>()
		)
		// With fewer than two samples between 10% and 80% the slope is NaN, which `== 0` let through.
		require(!(x.size < 2 || linReg[1].isInfinite() || linReg[1] >= 0)) {
			("Failed calibration: only " + x.size
					+ " samples while the robot sped up.")
		}
		this.tau = -1.0 / linReg[1]
	}
	
	companion object {
		var ALPHA_LARGE: Double = 10.2
		var ALPHA_SMALL: Double = 6.2
	}
}
