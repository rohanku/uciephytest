package uciephytest.phy

import chisel3._
import chisel3.util._

class DecoderIO(outBits: Int) extends Bundle {
  val binary = Input(UInt(log2Ceil(outBits + 1).W))
  val thermometer = Output(UInt(outBits.W))
}

class Decoder(outBits: Int) extends Module {
  val io = IO(new DecoderIO(outBits))
  
  io.thermometer := 0.U

  for (i <- 0 to outBits) {
    when (io.binary === i.U) {
      io.thermometer := ((BigInt(1) << i) - 1).U
    }
  }
}
