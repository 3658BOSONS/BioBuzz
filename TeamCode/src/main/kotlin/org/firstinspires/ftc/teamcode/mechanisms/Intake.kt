package org.firstinspires.ftc.teamcode.mechanisms

import org.firstinspires.ftc.teamcode.BuckoBee

import com.pedropathing.ivy.*

import dev.nextftc.robot.*
import dev.nextftc.hardware.actuators.*

class Intake : Mechanism {
	val combineIntake = NextMotor("combine")
	
	fun spin(powerSupplier: () -> Double): Command = infinite {
		this.combineIntake.throttle = powerSupplier()
	}
	
	fun stop(): Command = instant {
		this.combineIntake.throttle = 0.5;
	}
}

