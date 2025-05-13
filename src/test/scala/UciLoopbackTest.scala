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
import edu.berkeley.cs.ucie.digital.logphy.{
  LinkTrainingParams,
  TransmitPattern,
  RegisterRWIO,
  RegisterRW
}
import uciephytest.phy._

case object TesterParamsKey extends Field[TesterParams]

trait HasTesterParams {
  implicit val p: Parameters
  def tParams: TesterParams = p(TesterParamsKey)

  def lgBeatBytes = log2Up(tParams.beatBytes)
  def idBits = log2Up(tParams.maxInflight)
}

case class TesterParams (
  maxInflight: Int = 1,
  addrWidth: Int = 64,
  dataWidth: Int = 32,
  beatBytes: Int
)

abstract class TesterBundle(implicit val p: Parameters) extends ParameterizedBundle()(p) with HasTesterParams

class TesterReq(implicit p: Parameters) extends TesterBundle {
  val addr = Output(UInt(tParams.addrWidth.W))
  val data = Output(UInt(tParams.dataWidth.W))
  val id = Output(UInt(idBits.W))
  val is_write = Output(Bool())
}

class TesterResp(implicit p: Parameters) extends TesterBundle {
  val data = Output(UInt(tParams.dataWidth.W))
  val id = Output(UInt(idBits.W))
}

class TesterIO(implicit p: Parameters) extends TesterBundle {
  val req = Flipped(new DecoupledIO(new TesterReq))
  val resp = new ValidIO(new TesterResp)
}

class TileLinkTester(implicit p: Parameters) extends LazyModule with HasTesterParams {

  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLMasterParameters.v1(
    name = "tester-node",
    sourceId = IdRange(0, tParams.maxInflight) 
    ))
  )))
    
  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new TesterIO)

    val (out, edge) = node.out(0)

    io.req.ready := out.a.ready
    out.d.ready := true.B

    out.a.valid := io.req.valid
    // First arg is the source id
    out.a.bits := Mux(io.req.bits.is_write, 
                    edge.Put(io.req.bits.id, io.req.bits.addr, lgBeatBytes.U, io.req.bits.data)._2,
                    edge.Get(io.req.bits.id, io.req.bits.addr, lgBeatBytes.U)._2)

    io.resp.valid     := out.d.valid
    io.resp.bits.data := out.d.bits.data
    io.resp.bits.id   := out.d.bits.source
  }
}

class UciLoopbackTester(implicit p: Parameters) extends LazyModule {
  val params = uciephytest.UciephyTestParams(address=0x20000,
    numLanes = 16,
    tlParams = TileLinkParams(address = 0x580000000L,
    addressRange = (1L << 24) - 1,
    configAddress = 0x8000,
    inwardQueueDepth = 2,
    outwardQueueDepth = 2,
    dataWidth_arg = 256),
    onchipAddr = Some(0x1000000000L), sim = true)
  val delay = 0.0
  val txns = 100

  // Create clock source
  val clockSourceNode1 = ClockSourceNode(Seq(ClockSourceParameters()))
  val clockSourceNode2 = ClockSourceNode(Seq(ClockSourceParameters()))

  val regTester = LazyModule(new TileLinkTester)
  val ucieTester = LazyModule(new TileLinkTester)
  val phyTester = LazyModule(new TileLinkTester)
  val tlUcieDie1 = LazyModule(
    new UciephyTestTL(params = params, beatBytes = 16)
  )
  tlUcieDie1.clockNode := clockSourceNode1
  tlUcieDie1.uciTL.clockNode := clockSourceNode2
  tlUcieDie1.node := phyTester.node
  tlUcieDie1.uciTL.managerNode := TLSourceShrinker(
    params.tlParams.sourceIDWidth
  ) := ucieTester.node
  tlUcieDie1.uciTL.regNode.node := regTester.node
  val ram = LazyModule(
    new TLRAM(
      AddressSet(params.tlParams.ADDRESS, params.tlParams.addressRange),
      beatBytes = params.tlParams.BEAT_BYTES
    )
  )
  ram.node := tlUcieDie1.uciTL.clientNode
  // Extract UCIe module IO from Diplomacy
  val uciephyTestTLIO = BundleBridgeSink[uciephytest.UciephyTestTLIO]()
  uciephyTestTLIO := tlUcieDie1.topIO

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val uci_clock = Input(new ClockBundle(ClockBundleParameters()))
      val harness = new UciTLTestHarnessIO
    })

    regTester.module.io <> io.harness.regTester
    ucieTester.module.io <> io.harness.ucieTester
    phyTester.module.io <> io.harness.phyTester
    // connect IOs
    io.uci_clock <> clockSourceNode1.out(0)._1
    io.uci_clock <> clockSourceNode2.out(0)._1
    val ucieIO = uciephyTestTLIO.in(0)._1
    ucieIO.common.phy.refClkP := clock.asBool
    ucieIO.common.phy.refClkN := !clock.asBool
    ucieIO.common.phy.pllRdacVref := true.B
    ucieIO.common.phy.bypassClkP := clock.asBool
    ucieIO.common.phy.bypassClkN := !clock.asBool
    ucieIO.phy.rxClkP := ucieIO.phy.txClkP
    ucieIO.phy.rxClkN := ucieIO.phy.txClkN
    ucieIO.phy.rxValid := ucieIO.phy.txValid
    ucieIO.phy.rxTrack := ucieIO.phy.txTrack
    ucieIO.phy.rxData := ucieIO.phy.txData
    ucieIO.phy.sbRxClk := ucieIO.phy.sbTxClk
    ucieIO.phy.sbRxData := ucieIO.phy.sbTxData
  }
}

class UciTLTestHarnessIO(implicit val p: Parameters) extends Bundle {
    val regTester = new TesterIO
    val ucieTester = new TesterIO
    val phyTester = new TesterIO
}

class UciTLTestHarness(implicit val p: Parameters) extends Module {
  val io = IO(new UciTLTestHarnessIO)
  val tester = Module(LazyModule(new UciLoopbackTester).module)
  tester.io.harness.regTester <> io.regTester
  tester.io.harness.ucieTester <> io.ucieTester
  tester.io.harness.phyTester <> io.phyTester
  tester.io.uci_clock.clock := clock
  tester.io.uci_clock.reset := reset

  // Dummy plusarg to avoid breaking verilator builds with emulator.cc
  val useless_plusarg = PlusArg("useless_plusarg", width = 1)
  dontTouch(useless_plusarg)
  ElaborationArtefacts.add("plusArgs", PlusArgArtefacts.serialize_cHeader)
}

class UciLoopbackTest extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UciLoopback"
  val txns = 2
  val timeout = 100000
  implicit val p: Parameters = new Config((site, here, up) => {
    case TesterParamsKey => new TesterParams(maxInflight = 1, beatBytes = 16)
  })

  it should "finish request and response before timeout" in {
    test(new UciTLTestHarness()).withAnnotations(
      Seq(VcsBackendAnnotation, WriteVcdAnnotation)
    ) { c => 

      println("start Uci Loopback Test")
      c.reset.poke(true.B)
      c.clock.step(3)
      c.reset.poke(false.B)
      c.io.regTester.req.initSource()
      c.io.regTester.req.setSourceClock(c.clock)
      c.io.regTester.resp.initSink()
      c.io.regTester.resp.setSinkClock(c.clock)
      c.io.ucieTester.req.initSource()
      c.io.ucieTester.req.setSourceClock(c.clock)
      c.io.ucieTester.resp.initSink()
      c.io.ucieTester.resp.setSinkClock(c.clock)
      while (c.io.regTester.req.ready.peek().litValue != 1) {
        c.clock.step()
      }
      c.io.phyTester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "h22438".U,
        _.data -> 1.U,
        _.id -> 0.U,
        _.is_write -> true.B,
      ))
      c.clock.step(10)
      c.io.phyTester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "h224E8".U,
        _.data -> 1.U,
        _.id -> 1.U,
        _.is_write -> true.B,
      ))
      c.clock.step(10)
      c.io.ucieTester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "h1580000000".U,
        _.data -> 1.U,
        _.id -> 2.U,
        _.is_write -> true.B,
      ))
      c.clock.step(10)
      c.io.ucieTester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "h1580000000".U,
        _.data -> 0.U,
        _.id -> 3.U,
        _.is_write -> false.B,
      ))
      c.clock.step(10)
      c.clock.setTimeout(timeout + 10)
      c.io.ucieTester.resp.expectDequeue((new TesterResp).Lit(
        _.data -> 1.U,
        _.id -> 2.U,
      ))
    }
  }
}
