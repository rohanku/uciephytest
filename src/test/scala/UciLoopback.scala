package uciephytest

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters, Config}
import chisel3.experimental.hierarchy.{
  Definition,
  Instance,
  instantiable,
  public
}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.tilelink.{TLTestRAM}

import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{
  LinkTrainingParams,
  TransmitPattern,
  RegisterRWIO,
  RegisterRW
}

import uciephytest.phy._

// @instantiable
class UciLoopback(params: UciephyTestParams, beatBytes: Int) extends Module {
  val io = IO(new Bundle {
    // val finished = Output(Bool())
    // val mbAfe = Flipped(new MainbandAfeIo(afeParams))
    val test = Flipped(new PhyToTestIO(16))
    val sideband = Flipped(new SidebandIO)
  })

  val latency = 100
  /* val delayerMb = Module(new Pipe(chiselTypeOf(io.mbAfe.txData.bits),
   * latency)) */
  val delayerMbTx = Module(new Pipe(chiselTypeOf(io.test.tx.bits), latency))
  val delayerSb = Module(new Pipe(chiselTypeOf(io.sideband.txData), latency))
  val delayerSb_clock = Module(
    new Pipe(chiselTypeOf(io.sideband.txClk), latency)
  )
  // val delayerMbTx_clockn = Module(
  //   new Pipe(chiselTypeOf(io.mbAfe_tx.clkn.asBool), latency),
  // )
  // val delayerMbTx_clockp = Module(
  //   new Pipe(chiselTypeOf(io.mbAfe_tx.clkp.asBool), latency),
  // )

  /** TODO: ansa fix delayed signals -- not delayed */
  delayerMbTx.io.enq.valid := io.test.tx.valid
  delayerMbTx.io.enq.bits := io.test.tx.bits
  io.test.rx.bits := delayerMbTx.io.deq.bits
  io.test.rx.valid := delayerMbTx.io.deq.valid
  // delayerMbTx_clockn.io.enq.valid := true.B
  // delayerMbTx_clockn.io.enq.bits := io.mbAfe_tx.clkn.asBool
  // delayerMbTx_clockp.io.enq.valid := true.B
  // delayerMbTx_clockp.io.enq.bits := io.mbAfe_tx.clkp.asBool

  // io.mbAfe_rx.clkn := io.mbAfe_tx.clkn
  // io.mbAfe_rx.clkp := io.mbAfe_tx.clkp
  // io.mbAfe_rx.track := false.B

  delayerSb.io.enq.valid := true.B // io.sbAfe.txData.valid
  delayerSb.io.enq.bits := io.sideband.txData
  delayerSb_clock.io.enq.valid := true.B
  delayerSb_clock.io.enq.bits := io.sideband.txClk

  io.sideband.rxData := delayerSb.io.deq.bits
  val delayNegEdge = withClock((!clock.asBool).asClock)(RegInit(false.B))
  delayNegEdge := delayerSb_clock.io.deq.bits.asBool && delayerSb_clock.io.deq.valid
  io.sideband.rxClk := Mux(
    delayNegEdge,
    clock.asBool,
    false.B
  )
}
