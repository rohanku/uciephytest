package uciephytest.phy

import chisel3._
import chisel3.util._

class DriverControlIO extends Bundle {
  val en = Input(Bool()) 
  val en_b = Input(Bool()) 
}

class ClockingControlIO extends Bundle {
  // Use outputs for signals that do not need to be connected
  // val VCM = Output(Bool())
  // val in_buf = Output(Bits(8.W))
  // val inbm = Output(Bool())
  // val inbp = Output(Bool())
  // val injmb = Output(Bool())
  // val injpb = Output(Bool())
  // val mid = Output(Bits(8.W))
  // val mixer_out = Output(Bool())
  // val mixer_outb = Output(Bool())
  val misc = new MiscClockingControlIO
  val en = Input(Bits(64.W))
  val enb = Input(Bits(64.W))
}

class TxLaneIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val in = Input(Bool())
  val din = new TxDinIO
  val dout = Output(Bool())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val driver_ctl = new DriverControlIO
  val clocking_ctl = new ClockingControlIO
}

class TxLane extends RawModule {
  val io = IO(new TxLaneIO)

  val verilogBlackBox = Module(new VerilogTxLane)
  verilogBlackBox.io.injp := io.injp
  verilogBlackBox.io.injm := io.injm
  verilogBlackBox.io.in := io.in
  verilogBlackBox.io.resetb := io.resetb
  verilogBlackBox.io.din_0 :=  io.din.din_0
  verilogBlackBox.io.din_1 :=  io.din.din_1
  verilogBlackBox.io.din_2 :=  io.din.din_2
  verilogBlackBox.io.din_3 :=  io.din.din_3
  verilogBlackBox.io.din_4 :=  io.din.din_4
  verilogBlackBox.io.din_5 :=  io.din.din_5
  verilogBlackBox.io.din_6 :=  io.din.din_6
  verilogBlackBox.io.din_7 :=  io.din.din_7
  verilogBlackBox.io.din_8 :=  io.din.din_8
  verilogBlackBox.io.din_9 :=  io.din.din_9
  verilogBlackBox.io.din_10 := io.din.din_10
  verilogBlackBox.io.din_11 := io.din.din_11
  verilogBlackBox.io.din_12 := io.din.din_12
  verilogBlackBox.io.din_13 := io.din.din_13
  verilogBlackBox.io.din_14 := io.din.din_14
  verilogBlackBox.io.din_15 := io.din.din_15
  io.dout := verilogBlackBox.io.dout
  io.divclk := verilogBlackBox.io.divclk
  verilogBlackBox.io.driver_en := io.driver_ctl.en
  verilogBlackBox.io.driver_en_b := io.driver_ctl.en_b
  // io.clocking_ctl.VCM := verilogBlackBox.io.VCM
  // io.clocking_ctl.in_buf := verilogBlackBox.io.in_buf
  // io.clocking_ctl.inbm := verilogBlackBox.io.inbm
  // io.clocking_ctl.inbp := verilogBlackBox.io.inbp
  // io.clocking_ctl.injmb := verilogBlackBox.io.injmb
  // io.clocking_ctl.injpb := verilogBlackBox.io.injpb
  // io.clocking_ctl.mid := verilogBlackBox.io.mid
  // io.clocking_ctl.mixer_out := verilogBlackBox.io.mixer_out
  // io.clocking_ctl.mixer_outb := verilogBlackBox.io.mixer_outb
  // verilogBlackBox.io.band_ctrl := "b01".U(2.W)
  // verilogBlackBox.io.band_ctrlb := "b10".U(2.W)
  // verilogBlackBox.io.band_sel := io.clocking_ctl.misc.band_sel
  // verilogBlackBox.io.band_selb := io.clocking_ctl.misc.band_selb
  // verilogBlackBox.io.mix_en := 0.U(16.W)
  // verilogBlackBox.io.mix_enb := ~0.U(16.W)
  verilogBlackBox.io.mux_en := io.clocking_ctl.misc.mux_en
  verilogBlackBox.io.mux_enb := io.clocking_ctl.misc.mux_enb
  // verilogBlackBox.io.ph_sel := io.clocking_ctl.misc.ph_sel
  // verilogBlackBox.io.ph_selb := io.clocking_ctl.misc.ph_selb
  // verilogBlackBox.io.en := io.clocking_ctl.en
  // verilogBlackBox.io.enb := io.clocking_ctl.enb

  // val ctr = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U((log2Ceil(Phy.SerdesRatio) - 1).W)) }
  // ctr := ctr + 1.U

  // val divClock = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(false.B) }
  // when (ctr === 0.U) {
  //   divClock := !divClock
  // }

  // val shiftReg = withClockAndReset(io.injp.asClock, !io.resetb) { RegInit(0.U(Phy.SerdesRatio.W)) }
  // shiftReg := shiftReg >> 1.U
  // when (ctr === 0.U && !divClock) {
  //   shiftReg := io.din.asTypeOf(shiftReg)
  // }

  // io.divclk := divClock.asClock
  // io.dout := shiftReg(0)

  // io.driver_ctl <> 0.U.asTypeOf(io.driver_ctl)
  // io.clocking_ctl <> 0.U.asTypeOf(io.clocking_ctl)
}

class VerilogTxLaneIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val in = Input(Bool())
  val din_0 = Input(Bool())
  val din_1 = Input(Bool())
  val din_2 = Input(Bool())
  val din_3 = Input(Bool())
  val din_4 = Input(Bool())
  val din_5 = Input(Bool())
  val din_6 = Input(Bool())
  val din_7 = Input(Bool())
  val din_8 = Input(Bool())
  val din_9 = Input(Bool())
  val din_10 = Input(Bool())
  val din_11 = Input(Bool())
  val din_12 = Input(Bool())
  val din_13 = Input(Bool())
  val din_14 = Input(Bool())
  val din_15 = Input(Bool())
  val dout = Output(Bool())
  val divclk = Output(Clock())
  val resetb = Input(Bool())
  val driver_en = Input(Bool()) 
  val driver_en_b = Input(Bool()) 
  // val VCM = Output(Bool())
  // val in_buf = Output(Bits(8.W))
  // val inbm = Output(Bool())
  // val inbp = Output(Bool())
  // val injmb = Output(Bool())
  // val injpb = Output(Bool())
  // val mid = Output(Bits(8.W))
  // val mixer_out = Output(Bool())
  // val mixer_outb = Output(Bool())
  // val band_ctrl = Input(Bits(2.W))
  // val band_ctrlb = Input(Bits(2.W))
  // val band_sel = Input(Bits(2.W))
  // val band_selb = Input(Bits(2.W))
  // val mix_en = Input(Bits(16.W))
  // val mix_enb = Input(Bits(16.W))
  val mux_en = Input(Bits(8.W))
  val mux_enb = Input(Bits(8.W))
  // val ph_sel = Input(Bits(2.W))
  // val ph_selb = Input(Bits(2.W))
  // val en = Input(Bits(64.W))
  // val enb = Input(Bits(64.W))
}

class VerilogTxLane extends BlackBox {
  val io = IO(new VerilogTxLaneIO)

  override val desiredName = "TX_data_lane"
}

class TxClkIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val in = Input(Bool())
  val txckp = Output(Bool())
  val txckn = Output(Bool())
  val driver_ctl = Vec(2, new DriverControlIO)
  val VCM = Output(Bool())
  val inbm = Output(Bool())
  val inbp = Output(Bool())
  val injmb = Output(Bool())
  val injpb = Output(Bool())
  val mid = Output(Bits(8.W))
  val en = Input(Bits(64.W))
  val enb = Input(Bits(64.W))
}

class TxClk extends RawModule {
  val io = IO(new TxClkIO)

  val verilogBlackBox = Module(new VerilogTxClk)
  verilogBlackBox.io.injp := io.injp
  verilogBlackBox.io.injm := io.injm
  verilogBlackBox.io.in := io.in
  io.txckp := verilogBlackBox.io.txckp
  io.txckn := verilogBlackBox.io.txckn
  verilogBlackBox.io.driver0_en := io.driver_ctl(0).en
  verilogBlackBox.io.driver0_en_b := io.driver_ctl(0).en_b
  verilogBlackBox.io.driver1_en := io.driver_ctl(1).en
  verilogBlackBox.io.driver1_en_b := io.driver_ctl(1).en_b
  // io.VCM := verilogBlackBox.io.VCM
  // io.inbm := verilogBlackBox.io.inbm
  // io.inbp := verilogBlackBox.io.inbp
  // io.injmb := verilogBlackBox.io.injmb
  // io.injpb := verilogBlackBox.io.injpb
  // io.mid := verilogBlackBox.io.mid
  // verilogBlackBox.io.en := io.en
  // verilogBlackBox.io.enb := io.enb

  // io.txckp := io.injp
  // io.txckn := io.injm
  // 
  // io.driver0 <> 0.U.asTypeOf(io.driver0)
  // io.driver1 <> 0.U.asTypeOf(io.driver1)
  // io.driver0_en := true.B
  // io.driver1_en := true.B
  // io.driver0_en_b := false.B
  // io.driver1_en_b := false.B
}

class VerilogTxClkIO extends Bundle {
  val injp = Input(Bool())
  val injm = Input(Bool())
  val in = Input(Bool())
  val txckp = Output(Bool())
  val txckn = Output(Bool())
  val driver0_en = Input(Bool()) 
  val driver0_en_b = Input(Bool()) 
  val driver1_en = Input(Bool()) 
  val driver1_en_b = Input(Bool()) 
  // val VCM = Output(Bool())
  // val inbm = Output(Bool())
  // val inbp = Output(Bool())
  // val injmb = Output(Bool())
  // val injpb = Output(Bool())
  // val mid = Output(Bits(8.W))
  // val en = Input(Bits(64.W))
  // val enb = Input(Bits(64.W))
}

class VerilogTxClk extends BlackBox {
  val io = IO(new VerilogTxClkIO)

  override val desiredName = "TX_clk_tile"
}

class TxDriverIO extends Bundle {
  val din = Input(Bool())
  val dout = Output(Bool())
  val driver_ctl = new DriverControlIO
}

class TxDriver extends RawModule {
  val io = IO(new TxDriverIO)

  val verilogBlackBox = Module(new VerilogTxDriver)
  verilogBlackBox.io.clkin := io.din
  io.dout := verilogBlackBox.io.clkout
  verilogBlackBox.io.en := io.driver_ctl.en
  verilogBlackBox.io.en_b := io.driver_ctl.en_b

  // io.dout := io.din
}
class VerilogTxDriverIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val en = Input(Bool()) 
  val en_b = Input(Bool()) 
}

class VerilogTxDriver extends BlackBox {
  val io = IO(new VerilogTxDriverIO)

  override val desiredName = "txclk"
}
