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
  val resetb = Input(Bool())
  val zctl = noPrefix { new TerminationControlIO }
}

class RxLane extends RawModule {
  val io = noPrefix { IO(new RxLaneIO) }

  override val desiredName = "rxdata"

  val ctr = withClockAndReset(io.clk, !io.resetb) { RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W)) }
  ctr := ctr + 1.U

  val divClock = RegInit(false.B)
  when (ctr === 0.U) {
    divClock := !divClock
  }

  val shiftReg = withClockAndReset(io.clk, !io.resetb) { RegInit(0.U(Phy.SerdesRatio.W)) }
  shiftReg := shiftReg << 1.U | io.din.asUInt

  val outputReg = withClock(divClock.asClock) {
    RegNext(shiftReg)
  }

  io.divclk := divClock.asClock
  io.dout := outputReg
}

class RxClkIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val zctl = noPrefix { new TerminationControlIO }
}

class RxClk extends RawModule {
  val io = noPrefix { IO(new RxClkIO) }

  override val desiredName = "rxclk"
}
