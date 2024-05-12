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

class ClkDriverControlIO extends Bundle {
  val pu_ctl = noPrefix { new DriverPuControlIO }
  val pd_ctlb = noPrefix { new DriverPdControlIO }
}

class ClockingControlIO extends Bundle {
  // Use outputs for signals that do not need to be connected
  val NBIAS = Output(Bool())
  val VCM = Output(Bool())
  val in_buf = Output(Bits(8.W))
  val inbm = Output(Bool())
  val inbp = Output(Bool())
  val injmb = Output(Bool())
  val injpb = Output(Bool())
  val mid = Output(Bits(8.W))
  val mixer_out = Output(Bool())
  val mixer_outb = Output(Bool())
  val misc = noPrefix { new MiscClockingControlIO }
  val en = noPrefix { new EnClockingControlIO }
  val enb = noPrefix { new EnbClockingControlIO }
}

class TxLaneIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val in = Input(Bool())
  val din = noPrefix { new TxDinIO }
  val dout = Output(Bool())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val driver_ctl = noPrefix { new DriverControlIO }
  val clocking_ctl = noPrefix { new ClockingControlIO }
}

class TxLane extends RawModule {
  val io = noPrefix { IO(new TxLaneIO) }

  override val desiredName = "TX_data_lane"

  val ctr = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W)) }
  ctr := ctr + 1.U

  val divClock = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(false.B) }
  when (ctr === 0.U) {
    divClock := !divClock
  }

  val shiftReg = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U(Phy.SerdesRatio.W)) }
  shiftReg := shiftReg >> 1.U
  when (ctr === 0.U && !divClock) {
    shiftReg := io.din.asTypeOf(shiftReg)
  }

  io.divclk := divClock.asClock
  io.dout := shiftReg(0)

  io.driver_ctl <> 0.U.asTypeOf(io.driver_ctl)
  io.clocking_ctl <> 0.U.asTypeOf(io.clocking_ctl)
}

class TxClkIO extends Bundle {
  val txckp = Output(Bool())
  val txckn = Output(Bool())
  val driver0 = new ClkDriverControlIO
  val driver1 = new ClkDriverControlIO
  val driver0_en = Input(Bool()) 
  val driver0_en_b = Input(Bool()) 
  val driver1_en = Input(Bool()) 
  val driver1_en_b = Input(Bool()) 
  val VCM = Output(Bool())
  val en = noPrefix { new EnClockingControlIO }
  val enb = noPrefix { new EnbClockingControlIO }
  val in = Input(Bool())
  val injp = Input(Bool())
  val injm = Input(Bool())
  val inbm = Output(Bool())
  val inbp = Output(Bool())
  val injmb = Output(Bool())
  val injpb = Output(Bool())
  val mid = Output(Bits(8.W))
}

class TxClk extends RawModule {
  val io = noPrefix { IO(new TxClkIO) }

  override val desiredName = "TX_clk_tile"

  io.txckp := io.injp
  io.txckn := io.injm

  io.driver0 <> 0.U.asTypeOf(io.driver0)
  io.driver1 <> 0.U.asTypeOf(io.driver1)
  io.driver0_en := true.B
  io.driver1_en := true.B
  io.driver0_en_b := false.B
  io.driver1_en_b := false.B
}

class TxDriverIO extends Bundle {
  val din = Input(Bool())
  val dout = Output(Bool())
  val driver_ctl = noPrefix { new DriverControlIO }
}

class TxDriver extends RawModule {
  val io = noPrefix { IO(new TxDriverIO) }

  override val desiredName = "txdriver"

  io.dout := io.din
  io.driver_ctl <> 0.U.asTypeOf(io.driver_ctl)
}
