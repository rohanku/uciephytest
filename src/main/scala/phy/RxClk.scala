package uciephytest.phy

import chisel3._
import chisel3.util._
import chisel3.experimental.noPrefix

class RxClkIO extends Bundle {
  val clkin = Input(Bool())
  val clkout = Output(Bool())
  val zctl = noPrefix { new TerminationControlIO }
}

class RxClk extends RawModule {
  val io = noPrefix { IO(new RxClkIO) }

  override val desiredName = "rxclk"
}
