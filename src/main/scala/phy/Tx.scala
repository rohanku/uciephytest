package uciephytest.phy

import chisel3._
import chisel3.util._

class TxLaneIO extends Bundle {
  // Pull-up impedance control.
  val driverPuCtl = Input(UInt(8.W))
  // Pull-down impedance control.
  val driverPdCtl = Input(UInt(8.W))
  // Driver enable signal. When low, the driver enters a high-Z state.
  val driverEn = Input(Bool()) 
  // Phase control per lane.
  val phaseCtl = Input(UInt(8.W))

  val divClock = Output(Clock())
  val din = Input(UInt(Phy.SerdesRatio.W))
  val dout = Output(Bool())
}

class TxLane extends Module {
  val io = IO(new TxLaneIO)

  // TODO: Fix behavioral model
  io.divClock := clock
  io.dout := false.B
}
