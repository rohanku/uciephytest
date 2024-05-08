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

  // TODO: Fix behavioral model
  io.divClock := clock
  io.dout := 0.U
}
