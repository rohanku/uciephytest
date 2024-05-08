package uciephytest.phy

import chisel3._
import chisel3.util._

class RxLaneIO extends Bundle {
  // TODO: Add termination control and other control pins
  val divClock = Output(Clock())
  val din = Input(Bool())
  val dout = Output(UInt(Phy.SerdesRatio.W))
}

class RxLane extends Module {
  val io = IO(new RxLaneIO)

  val ctr = RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W))
  ctr := ctr + 1.U

  val divClock = RegInit(false.B)
  when (ctr === 0.U) {
    divClock = !divClock
  }

  val shiftReg = RegInit(0.U(Phy.SerdesRatio.W))
  shiftReg := shiftReg << 1.U + din.asUInt

  val outputReg = withClock(divClock) {
    RegNext(shiftReg)
  }

  io.divClock := divClock
  io.dout := outputReg
}
