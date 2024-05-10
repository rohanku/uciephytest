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

class ClockingControlIO extends Bundle {
  // Use outputs for signals that do not need to be connected
  val NBIAS = Output(Bool())
  val VCM = Output(Bool())
  val in_buf_0 = Output(Bool()).suggestName("in_buf<0>")
  val in_buf_1 = Output(Bool()).suggestName("in_buf<1>")
  val in_buf_2 = Output(Bool()).suggestName("in_buf<2>")
  val in_buf_3 = Output(Bool()).suggestName("in_buf<3>")
  val in_buf_4 = Output(Bool()).suggestName("in_buf<4>")
  val in_buf_5 = Output(Bool()).suggestName("in_buf<5>")
  val in_buf_6 = Output(Bool()).suggestName("in_buf<6>")
  val in_buf_7 = Output(Bool()).suggestName("in_buf<7>")
  val in = Input(Bool())
  val inbm = Output(Bool())
  val inbp = Output(Bool())
  val injmb = Output(Bool())
  val injpb = Output(Bool())
  val mid_0 = Output(Bool()).suggestName("mid<0>")
  val mid_1 = Output(Bool()).suggestName("mid<1>")
  val mid_2 = Output(Bool()).suggestName("mid<2>")
  val mid_3 = Output(Bool()).suggestName("mid<3>")
  val mid_4 = Output(Bool()).suggestName("mid<4>")
  val mid_5 = Output(Bool()).suggestName("mid<5>")
  val mid_6 = Output(Bool()).suggestName("mid<6>")
  val mid_7 = Output(Bool()).suggestName("mid<7>")
  val misc = noPrefix { new MiscClockingControlIO }
  val en = noPrefix { new EnClockingControlIO }
  val enb = noPrefix { new EnbClockingControlIO }
}

class TxLaneIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val din = noPrefix { new TxDinIO }
  val dout = Output(Bool())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val driver_ctl = noPrefix { new DriverControlIO }
  val clocking_ctl = noPrefix { new ClockingControlIO }
}

class TxLane extends RawModule {
  val io = noPrefix { IO(new TxLaneIO) }

  override val desiredName = "txdata"

  val ctr = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W)) }
  ctr := ctr + 1.U

  val divClock = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(false.B) }
  when (ctr === 0.U) {
    divClock := !divClock
  }

  val shiftReg = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U(Phy.SerdesRatio.W)) }
  shiftReg := shiftReg << 1.U
  when (ctr === 0.U && !divClock) {
    shiftReg := io.din
  }

  io.divclk := divClock.asClock
  io.dout := shiftReg(Phy.SerdesRatio - 1)
}

class TxClkIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val clkout = Output(Bool())
  val clkoutb = Output(Bool())
  val driver_ctl = Vec(2, new DriverControlIO)
  val clocking_ctl = noPrefix { new ClockingControlIO }
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
