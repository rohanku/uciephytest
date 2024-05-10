package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.noPrefix

class DriverControlIO extends Bundle {
  val pu_ctl = noPrefix { new DriverPuControlIO }
  val pd_ctlb = noPrefix { new DriverPdControlIO }
  val driver_en = Input(Bool()) 
  val driver_en_b = Input(Bool()) 
}

class TxLaneIO extends Bundle {
  val clk = Input(Bool())
  val clkb = Input(Bool())
  val din = noPrefix { new TxDinIO }
  val dout = Output(Bool())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val driver_ctl = noPrefix { new DriverControlIO }
}

class TxLane extends RawModule {
  val io = noPrefix { IO(new TxLaneIO) }

  override val desiredName = "txdata"

  val ctr = RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W))
  ctr := ctr + 1.U

  val divClock = RegInit(false.B)
  when (ctr === 0.U) {
    divClock := !divClock
  }

  val shiftReg = RegInit(0.U(Phy.SerdesRatio.W))
  shiftReg := shiftReg << 1.U
  when (ctr === 0.U && !divClock) {
    shiftReg := io.din
  }

  io.divclk := divClock.asClock
  io.dout := shiftReg(Phy.SerdesRatio - 1)
}

class TxClkIO extends Bundle {
  val clk = Input(Bool())
  val clkb = Input(Bool())
  val clkout = Output(Bool())
  val clkoutb = Output(Bool())
  val driver_ctl = Vec(2, new DriverControlIO)
}

class TxClk extends RawModule {
  val io = noPrefix { IO(new TxClkIO) }

  override val desiredName = "txclk"
}

class TxDriverIO extends Bundle {
  val din = Input(Bool())
  val dout = Output(Bool())
  val driver_ctl = noPrefix { new DriverControlIO }
}

class TxDriver extends RawModule {
  val io = noPrefix { IO(new TxDriverIO) }

  override val desiredName = "txdriver"
}
