package uciephytest

import chisel3._
import chisel3.util._
import uciephytest.phy.{Phy, RxIO}
import edu.berkeley.cs.ucie.digital.interfaces.AfeParams

class ValidFramerIO(
  afeParams: AfeParams
) extends Bundle {
  // UCIE DIGITAL INTERFACE
  val digital = Decoupled(Vec(afeParams.mbLanes, Bits(afeParams.mbSerializerRatio.W)))

  // PHY INTERFACE
  // ====================
  val phy = Flipped(DecoupledIO(new RxIO(afeParams.mbLanes)))
}

class ValidFramer(
  afeParams: AfeParams,
) extends Module {
  val io = IO(new ValidFramerIO(afeParams))

  // RX logic
  io.phy.ready := true.B
  io.digital.valid := false.B
  io.digital.bits := 0.U.asTypeOf(io.digital.bits)

  val serializerLenBits = log2Ceil(afeParams.mbSerializerRatio)

  val hasOne = Wire(Bool())
  val firstOne = Wire(UInt(serializerLenBits.W))
  hasOne := false.B
  firstOne := 0.U

  // afeParams.mbLanes data lanes, 1 valid lane.
  val runningData = RegInit(VecInit(Seq.fill(afeParams.mbLanes + 1)(0.U(afeParams.mbSerializerRatio.W))))

  // Check valid streak after each packet is dequeued.
  when(io.phy.ready & io.phy.valid) {
    // Store latest data at the beginning of the `runningData` register.
    val nextData = Wire(Vec(afeParams.mbLanes + 1, UInt((2 * afeParams.mbSerializerRatio).W)))
    for (lane <- 0 until afeParams.mbLanes + 1) {
      if (lane < afeParams.mbLanes) {
        nextData(lane) := Cat(io.phy.bits.data(lane), runningData(lane))
      } else {
        nextData(lane) := Cat(io.phy.bits.valid, runningData(lane))
      }
      runningData(lane) := nextData(lane)(2 * afeParams.mbSerializerRatio - 1, afeParams.mbSerializerRatio)
    }


    // Find first one.
    hasOne := nextData(afeParams.mbLanes)(afeParams.mbSerializerRatio - 1, 0).orR
    for (i <- afeParams.mbSerializerRatio - 1 to 0 by -1) {
      when(nextData(afeParams.mbLanes)(i)) {
        firstOne := i.U
      }
    }

    when (hasOne) {
      io.digital.valid := true.B
      for (lane <- 0 until afeParams.mbLanes) {
        io.digital.bits(lane) := (nextData(lane) >> firstOne)(afeParams.mbSerializerRatio - 1, 0)
      }
    }
  }
}
