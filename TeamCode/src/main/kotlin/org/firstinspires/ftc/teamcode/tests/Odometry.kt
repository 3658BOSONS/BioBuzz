package org.firstinspires.ftc.teamcode.tests

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "Manual Three Wheel Tuner", group = "Calibration")
class ManualThreeWheelTuner : LinearOpMode() {
	
	companion object {
		private const val TAG = "OdometryTuner"
		
		private const val LEFT_ENCODER_NAME = "frontLeftMotor"
		private const val RIGHT_ENCODER_NAME = "backRightMotor"
		private const val STRAFE_ENCODER_NAME = "frontRightMotor"
		
		// Drive motors (set to FLOAT so you can push the robot smoothly)
		private val DRIVE_MOTOR_NAMES = listOf("frontLeftMotor", "backLeftMotor", "frontRightMotor", "backRightMotor")
		
		// 2. SET YOUR PUSH DISTANCE IN INCHES
		private const val PUSH_DISTANCE_INCHES = 48.0
	}
	
	private enum class Step {
		PUSH_FORWARD,
		PUSH_STRAFE,
		ROTATE_360,
		DONE
	}
	
	override fun runOpMode() {
		// Enable bulk caching for accurate reads
		for (hub in hardwareMap.getAll(LynxModule::class.java)) {
			hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
		}
		
		// Float all drive motors so pushing/rotating has no resistance
		for (name in DRIVE_MOTOR_NAMES) {
			val motor = hardwareMap.tryGet(DcMotorEx::class.java, name)
			motor?.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
			motor?.power = 0.0
		}
		
		val leftPod = hardwareMap.get(DcMotorEx::class.java, LEFT_ENCODER_NAME)
		val rightPod = hardwareMap.get(DcMotorEx::class.java, RIGHT_ENCODER_NAME)
		val strafePod = hardwareMap.get(DcMotorEx::class.java, STRAFE_ENCODER_NAME)
		
		var currentStep = Step.PUSH_FORWARD
		var prevA = false
		var prevB = false
		
		// Zero baselines
		var baseL = leftPod.currentPosition
		var baseR = rightPod.currentPosition
		var baseS = strafePod.currentPosition
		
		// Recorded values
		var forwardTicksToInches = 0.0
		var strafeTicksToInches = 0.0
		var leftDir = "Encoder.FORWARD"
		var rightDir = "Encoder.FORWARD"
		var strafeDir = "Encoder.FORWARD"
		
		var leftPodY = 0.0
		var rightPodY = 0.0
		var strafePodX = 0.0
		var turnTicksToRadians = 0.0
		
		telemetry.addLine("Ready! Press PLAY, then follow on-screen instructions.")
		telemetry.update()
		
		waitForStart()
		
		// Reset baselines right after Play is pressed
		baseL = leftPod.currentPosition
		baseR = rightPod.currentPosition
		baseS = strafePod.currentPosition
		
		while (opModeIsActive()) {
			val dL = leftPod.currentPosition - baseL
			val dR = rightPod.currentPosition - baseR
			val dS = strafePod.currentPosition - baseS
			
			val aPressed = gamepad1.a && !prevA
			val bPressed = gamepad1.b && !prevB
			prevA = gamepad1.a
			prevB = gamepad1.b
			
			// Pressing B at any step zeroes the current encoder deltas
			if (bPressed && currentStep != Step.DONE) {
				baseL = leftPod.currentPosition
				baseR = rightPod.currentPosition
				baseS = strafePod.currentPosition
				Log.i(TAG, "Baselines reset during step: $currentStep")
			}
			
			when (currentStep) {
				Step.PUSH_FORWARD -> {
					telemetry.addLine("=== STEP 1: FORWARD RESOLUTION ===")
					telemetry.addLine("1. Push robot FORWARD exactly $PUSH_DISTANCE_INCHES inches.")
					telemetry.addLine("2. Press [A] to record | Press [B] to zero ticks.")
					telemetry.addData("Left Delta", dL)
					telemetry.addData("Right Delta", dR)
					
					if (aPressed) {
						if (dL == 0 || dR == 0) {
							telemetry.addLine("ERROR: One or both forward encoders read 0!")
						} else {
							val leftTpi = abs(dL.toDouble() / PUSH_DISTANCE_INCHES)
							val rightTpi = abs(dR.toDouble() / PUSH_DISTANCE_INCHES)
							val avgForwardTpi = 2.0 / ((1.0 / leftTpi) + (1.0 / rightTpi))
							
							forwardTicksToInches = 1.0 / avgForwardTpi
							leftDir = if (dL < 0) "Encoder.REVERSE" else "Encoder.FORWARD"
							rightDir = if (dR < 0) "Encoder.REVERSE" else "Encoder.FORWARD"
							
							Log.i(TAG, "Step 1 Done -> forwardTicksToInches=$forwardTicksToInches, leftDir=$leftDir, rightDir=$rightDir")
							
							// Reset for next step
							baseL = leftPod.currentPosition
							baseR = rightPod.currentPosition
							baseS = strafePod.currentPosition
							currentStep = Step.PUSH_STRAFE
						}
					}
				}
				
				Step.PUSH_STRAFE -> {
					telemetry.addLine("=== STEP 2: STRAFE RESOLUTION ===")
					telemetry.addLine("1. Push robot LEFT exactly $PUSH_DISTANCE_INCHES inches.")
					telemetry.addLine("2. Press [A] to record | Press [B] to zero ticks.")
					telemetry.addData("Strafe Delta", dS)
					
					if (aPressed) {
						if (dS == 0) {
							telemetry.addLine("ERROR: Strafe encoder read 0!")
						} else {
							val strafeTpi = abs(dS.toDouble() / PUSH_DISTANCE_INCHES)
							strafeTicksToInches = 1.0 / strafeTpi
							strafeDir = if (dS < 0) "Encoder.REVERSE" else "Encoder.FORWARD"
							
							Log.i(TAG, "Step 2 Done -> strafeTicksToInches=$strafeTicksToInches, strafeDir=$strafeDir")
							
							// Reset for next step
							baseL = leftPod.currentPosition
							baseR = rightPod.currentPosition
							baseS = strafePod.currentPosition
							currentStep = Step.ROTATE_360
						}
					}
				}
				
				Step.ROTATE_360 -> {
					// Apply detected directions so turning CCW yields expected signs
					val signedDL = if (leftDir == "Encoder.REVERSE") -dL else dL
					val signedDR = if (rightDir == "Encoder.REVERSE") -dR else dR
					val signedDS = if (strafeDir == "Encoder.REVERSE") -dS else dS
					
					telemetry.addLine("=== STEP 3: OFFSETS & TURN MULTIPLIER ===")
					telemetry.addLine("1. Rotate robot COUNTERCLOCKWISE 360 degrees in place.")
					telemetry.addLine("2. Press [A] to finish | Press [B] to zero ticks.")
					telemetry.addData("Signed L / R / S", "$signedDL / $signedDR / $signedDS")
					
					if (aPressed) {
						val totalAngleRad = 2.0 * PI
						
						// Distance traveled by each pod in inches during 360 CCW spin
						val leftInches = signedDL * forwardTicksToInches
						val rightInches = signedDR * forwardTicksToInches
						val strafeInches = signedDS * strafeTicksToInches
						
						// Calculate track width / pod offsets from center of rotation
						// In Pedro Pathing (CCW = +angle): left pod moves backward (-), right moves forward (+)
						leftPodY = -leftInches / totalAngleRad
						rightPodY = -rightInches / totalAngleRad
						strafePodX = strafeInches / totalAngleRad
						
						// Turn multiplier: radians per differential encoder tick
						val diffTicks = abs(signedDR - signedDL).toDouble()
						turnTicksToRadians = if (diffTicks > 0) totalAngleRad / diffTicks else forwardTicksToInches
						
						val configCode = """
                            public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {
                                c.leftEncoderName.set("$LEFT_ENCODER_NAME");
                                c.rightEncoderName.set("$RIGHT_ENCODER_NAME");
                                c.strafeEncoderName.set("$STRAFE_ENCODER_NAME");
                                c.leftPodY.set($leftPodY);
                                c.rightPodY.set($rightPodY);
                                c.strafePodX.set($strafePodX);
                                c.forwardTicksToInches.set($forwardTicksToInches);
                                c.strafeTicksToInches.set($strafeTicksToInches);
                                c.turnTicksToRadians.set($turnTicksToRadians);
                                c.leftEncoderDirection.set($leftDir);
                                c.rightEncoderDirection.set($rightDir);
                                c.strafeEncoderDirection.set($strafeDir);
                            });
                        """.trimIndent()
						
						Log.i(TAG, "=== CALIBRATION COMPLETE ===\n$configCode")
						currentStep = Step.DONE
					}
				}
				
				Step.DONE -> {
					telemetry.addLine("=== CALIBRATION COMPLETE! ===")
					telemetry.addLine("Check Logcat (Tag: OdometryTuner) to copy-paste full code.")
					telemetry.addData("leftPodY", "%.4f".format(leftPodY))
					telemetry.addData("rightPodY", "%.4f".format(rightPodY))
					telemetry.addData("strafePodX", "%.4f".format(strafePodX))
					telemetry.addData("forwardTicksToInches", "%.6f".format(forwardTicksToInches))
					telemetry.addData("strafeTicksToInches", "%.6f".format(strafeTicksToInches))
					telemetry.addData("turnTicksToRadians", "%.6f".format(turnTicksToRadians))
					telemetry.addData("Directions (L/R/S)", "$leftDir / $rightDir / $strafeDir")
				}
			}
			
			telemetry.update()
		}
	}
}