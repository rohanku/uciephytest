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
    onchipAddr = Some(0x1000000000L))
  val delay = 0.0
  val txns = 100

  // Create clock source
  val clockSourceNode = ClockSourceNode(Seq(ClockSourceParameters()))

  val tester = LazyModule(new TileLinkTester)
  val tlUcieDie1 = LazyModule(
    new UciephyTestTL(params = params, beatBytes = 16)
  )
  tlUcieDie1.clockNode := clockSourceNode
  tlUcieDie1.node := tester.node
  tlUcieDie1.uciTL.managerNode := TLSourceShrinker(
    params.tlParams.sourceIDWidth
  ) := tester.node
  ram.node := tlUcieDie1.uciTL.clientNode
  val ram = LazyModule(
    new TLRAM(
      AddressSet(params.tlParams.ADDRESS, params.tlParams.addressRange),
      beatBytes = params.tlParams.BEAT_BYTES
    )
  )
  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val uci_clock = Input(new ClockBundle(ClockBundleParameters()))
      val tester = new TesterIO
    })

    tester.io.out(0)._1 <> io.tester
    // connect IOs
    io.uci_clock <> clockSourceNode.out(0)._1
    val ucieIO = tlUcieDie1.out(0)._1
    phy.io.common.refClkP := clock
    phy.io.common.refClkN := (!clock.asBool).asClock
    phy.io.common.pllRdacVref := true.B
    phy.io.common.bypassClkP := clock
    phy.io.common.bypassClkN := (!clock.asBool).asClock
    phy.io.top.rxClkP := phy.io.top.txClkP
    phy.io.top.rxClkN := phy.io.top.txClkN
    phy.io.top.rxData := phy.io.top.txData
    phy.io.top.rxValid := phy.io.top.txValid
    phy.io.top.rxTrack := phy.io.top.txTrack
    phy.io.top.sbRxData := phy.io.top.sbTxData
    phy.io.top.sbRxClk := phy.io.top.sbTxClk
    phy.io.sideband.txData := false.B
    phy.io.sideband.txClk := false.B

  }
}

class UciTLTestHarness(implicit val p: Parameters) extends Module {
  val io = IO(new Bundle { 
    val success = Output(Bool()) 
    val tester = new TesterIO
  })
  val tester = Module(LazyModule(new UciLoopbackTester).module)
  tester.io <> io.tester
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
      Seq(VcsBackendAnnotation, WriteFsdbAnnotation)
    ) { c => // .withAnnotations(Seq(VcsBackendAnnotation, WriteVcdAnnotation))

      println("start Uci Loopback Test")
      c.reset.poke(true.B)
      c.clock.step(3)
      c.reset.poke(false.B)
      c.io.tester.req.initSource()
      c.io.tester.req.setSourceClock(c.clock)
      c.io.tester.resp.initSink()
      c.io.tester.resp.setSourceClock(c.clock)
      c.io.tester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "h22438".U,
        _.data -> 1.U,
        _.id -> 0.U,
        _.is_write -> true.B,
      ))
      c.clock.step(10)
      c.io.tester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "1580000000".U,
        _.data -> 1.U,
        _.id -> 1.U,
        _.is_write -> true.B,
      ))
      c.clock.step(10)
      c.io.tester.req.enqueueNow((new TesterReq).Lit(
        _.addr -> "1580000000".U,
        _.data -> 0.U,
        _.id -> 2.U,
        _.is_write -> false.B,
      ))
      c.clock.step(10)
      c.io.tester.resp.expectDequeue((new TesterResp).Lit(
        _.data -> 1.U,
        _.id -> 2.U,
      ))
      c.clock.setTimeout(timeout + 10)
      c.clock.step(timeout)
      c.io.success.expect(true.B)
      println("Uci Loopback Test finished? " + c.io.success.peek())
    }
  }
}
