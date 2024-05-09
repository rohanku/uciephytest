package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.noPrefix

class RxLaneIO extends Bundle {
  val din = Input(Bool())
  val dout = noPrefix { new RxDoutIO }
  val clk = Input(Clock())
  val clkb = Input(Clock())
  val divclk = Output(Clock())
  val resetb = Input(Clock())
  val terminationCtl = new TerminationControlIO
}

class RxLane extends RawModule {
  val io = noPrefix { IO(new RxLaneIO) }

  val ctr = RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W))
  ctr := ctr + 1.U

  val divClock = RegInit(false.B)
  when (ctr === 0.U) {
    divClock := !divClock
  }

  val shiftReg = RegInit(0.U(Phy.SerdesRatio.W))
  shiftReg := shiftReg << 1.U | io.din.asUInt

  val outputReg = withClock(divClock.asClock) {
    RegNext(shiftReg)
  }

  io.divClock := divClock.asClock
  io.dout := outputReg
}
