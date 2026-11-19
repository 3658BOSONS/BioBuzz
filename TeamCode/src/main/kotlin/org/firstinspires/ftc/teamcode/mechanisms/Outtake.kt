package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import org.firstinspires.ftc.teamcode.BuckoBee

import dev.nextftc.robot.*
import dev.nextftc.hardware.actuators.*

class Outtake : Mechanism {
	val flippy = NextServo("flippy")
	val grippy = NextServo("grippy")
	
	fun up(): Command = instant {
		this.flippy.position = 0.5
	}
	
	fun down(): Command = instant {
		this.flippy.position = 0.0
	}
	
	fun grab(): Command = instant {
		this.grippy.position = 0.25
	}
	
	fun drop(): Command = instant {
		this.grippy.position = 0.0
	}
}