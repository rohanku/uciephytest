package uciephytest

import chisel3._
import chisel3.util._
import chiseltest._
import org.chipsalliance.cde.config.{Field, Parameters, Config}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.prci._
import freechips.rocketchip.diplomacy._
import chisel3.experimental.BundleLiterals._

// import freechips.rocketchip.unittest._
import org.scalatest.flatspec.AnyFlatSpec
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{LinkTrainingParams, TransmitPattern, RegisterRWIO, RegisterRW}

import uciephytest.phy._

import java.util.ResourceBundle

class UciLoopbackTester(implicit p: Parameters) extends LazyModule {
  val params = UciephyTestParams()
  val delay = 0.0
  val txns = 100

  // Create clock source
  val clockSourceNode = ClockSourceNode(Seq(ClockSourceParameters()))

  //val csrfuzz = LazyModule(new TLFuzzer(txns))
  val fuzz = LazyModule(new TLFuzzer(txns))
  val tlUcieDie1 = LazyModule(new UciephyTestTL(
     params = params,
     beatBytes = 16))
  tlUcieDie1.clockNode := clockSourceNode
  val ram = LazyModule(
    new TLRAM(
      AddressSet(params.tlParams.ADDRESS, params.tlParams.addressRange),
      beatBytes = params.tlParams.BEAT_BYTES,
    ),
  )

  // CSR node
  //tlUcieDie1.regNode.node := csrfuzz.node
  // connect data nodes
  tlUcieDie1.uciTL.managerNode := TLSourceShrinker(
    params.tlParams.sourceIDWidth,
  ) := fuzz.node
  ram.node := tlUcieDie1.uciTL.clientNode
  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val uci_clock = Input(new ClockBundle(ClockBundleParameters()))
      val finished = Output(Bool())
    })
    // connect IOs
    io.finished := fuzz.module.io.finished
    val UciLoopback = Module(new UciLoopback(params, 16))
    io.uci_clock <> clockSourceNode.out(0)._1
    // inputs to tlUcieDie1
    // tlUcieDie1.module.io.mbAfe <> AfeLoopback.io.mbAfe
    // tlUcieDie1.module.phy.io.test.tx <> UciLoopback.io.test.tx
    // tlUcieDie1.module.phy.io.test.rx <> UciLoopback.io.test.rx
    // tlUcieDie1.module.phy.io.sideband.txData <> UciLoopback.io.sideband.txData
    // tlUcieDie1.module.phy.io.sideband.txClk <> UciLoopback.io.sideband.txClk
    // tlUcieDie1.module.phy.io.sideband.rxData <> UciLoopback.io.sideband.rxData
    // tlUcieDie1.module.phy.io.sideband.rxClk <> UciLoopback.io.sideband.rxClk

  }
}

class UciTLTestHarness(implicit val p: Parameters) extends Module {
  val io = IO(new Bundle { val success = Output(Bool()) })
  val tester = Module(LazyModule(new UciLoopbackTester).module)
  tester.io.uci_clock.clock := clock
  tester.io.uci_clock.reset := reset
  io.success := tester.io.finished

  // Dummy plusarg to avoid breaking verilator builds with emulator.cc
  val useless_plusarg = PlusArg("useless_plusarg", width = 1)
  dontTouch(useless_plusarg)
  ElaborationArtefacts.add("plusArgs", PlusArgArtefacts.serialize_cHeader)
}

class UciLoopbackTest extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UciLoopback"
  val txns = 2
  val timeout = 100000
  implicit val p: Parameters = Parameters.empty
  it should "finish request and response before timeout" in {
    test(new UciTLTestHarness()).withAnnotations(
      Seq(VcsBackendAnnotation, WriteFsdbAnnotation),
    ) { c => // .withAnnotations(Seq(VcsBackendAnnotation, WriteVcdAnnotation))

      println("start Uci Loopback Test")
      c.reset.poke(true.B)
      c.clock.step(3)
      c.reset.poke(false.B)
      c.clock.setTimeout(timeout + 10)
      c.clock.step(timeout)
      c.io.success.expect(true.B)
      println("Uci Loopback Test finished? " + c.io.success.peek())
    }
  }
}
