package uciephytest

import chisel3._
import chisel3.util._

import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.{Field, Parameters, Config}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.prci._
import edu.berkeley.cs.ucie.digital.tilelink._

import tlt._

/* 
make MODEL=TLUCIeTestHarness MODEL_PACKAGE=uciephytest CONFIG=UCIeTLTConfig CONFIG_PACKAGE=uciephytest BINARY=$RISCV/riscv64-unknown-elf/share/riscv-tests/isa/rv64ui-p-simple run-binary-debug TOP=TLUCIeTester EXTRA_SIM_FLAGS='+tltestfile=/scratch/schwarzem/kodiak-ucie/tools/tilelink-tester/src/main/resources/tilelink-tester/python/example/example_0_strided_random_10_256'
*/

trait HasTesterSuccessIO { this: Module =>
  val io = IO(new Bundle {
    val success = Output(Bool())
  })
}

class TLUCIeTester(implicit p: Parameters) extends LazyModule {
  val testerParams = p(TesterParamsKey)
  val ucieParams = p(UciephyTestKey).get(0)

  val placeholder_node_ucie_regmap = TLClientNode(Seq(
    TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
      name = "placeholder-node-ucie-regmap",
      sourceId = IdRange(0, testerParams.maxInflight) 
    ))),
  ))

  val placeholder_node_tl_regmap = TLClientNode(Seq(
    TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
      name = "placeholder-node-tl-regmap",
      sourceId = IdRange(0, testerParams.maxInflight) 
    )))
  ))

  val tltester = LazyModule(new TileLinkTester)

  val ucie = LazyModule(new UciephyTestTL(ucieParams, testerParams.beatBytes)(p))

  val mem = LazyModule(new TLRAM(AddressSet(ucieParams.tlParams.address, ucieParams.tlParams.addressRange)))

  val clockSourceNode_digital = ClockSourceNode(Seq(ClockSourceParameters())) // drive uciephy and ucietl clock nodes
  val clockSourceNode_phy     = ClockSourceNode(Seq(ClockSourceParameters()))

  ucie.uciTL.managerNode := tltester.node
  mem.node := ucie.uciTL.clientNode

  ucie.clockNode := clockSourceNode_digital
  ucie.uciTL.clockNode := clockSourceNode_phy

  ucie.node := placeholder_node_ucie_regmap
  ucie.uciTL.regNode.node := placeholder_node_tl_regmap

  val uciephyTopIO = BundleBridgeSink[uciephytest.UciephyTopIO]()
  uciephyTopIO := ucie.topIO

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val system_clock = Input(new ClockBundle(ClockBundleParameters()))
      val done = Output(Bool())
    })

    val (out_ucie, _) = placeholder_node_ucie_regmap.out(0)
    val (out_tl, _) = placeholder_node_tl_regmap.out(0)
    out_ucie.tieoff()
    out_tl.tieoff()

    val driver = Module(new TLTesterDriver(testerParams.addrWidth, testerParams.dataWidth, log2Up(testerParams.maxInflight), testerParams.maxInflight)) 

    driver.io.tlt <> tltester.module.io
    driver.io.clock := clock
    driver.io.reset := reset

    io.done := driver.io.done

    // Get module IO from bundle bridge sink
    val uciphyIO_0 = uciephyTopIO.in(0)._1

    // Loopback
    uciphyIO_0.refClkP := clock
    uciphyIO_0.refClkN := (!clock.asBool).asClock
    uciphyIO_0.rxClkP := uciphyIO_0.txClkP
    uciphyIO_0.rxClkN := uciphyIO_0.txClkN
    uciphyIO_0.pllIref := false.B
    uciphyIO_0.rxValid := uciphyIO_0.txValid
    uciphyIO_0.rxtrk := uciphyIO_0.txtrk
    uciphyIO_0.rxData := uciphyIO_0.txData
    uciphyIO_0.sbRxClk := uciphyIO_0.sbTxClk
    uciphyIO_0.sbRxData := uciphyIO_0.sbTxData

    clockSourceNode_digital.out(0)._1 <> io.system_clock
    clockSourceNode_phy.out(0)._1     <> io.system_clock
  }
}

class TLUCIeTestHarness(implicit val p: Parameters) extends Module with HasTesterSuccessIO {
  val tester = Module(LazyModule(new TLUCIeTester).module)
  tester.io.system_clock.clock := clock
  tester.io.system_clock.reset := reset
  io.success := tester.io.done
}

class UCIeTLTConfig extends Config(
  new WithUciephyTest(Seq(UciephyTestParams(address=0x4000,
                                            numLanes = 16,
                                            tlParams = TileLinkParams(address = 0x0L,
                                            addressRange = (1L << 32) - 1,
                                            configAddress = 0x8000,
                                            inwardQueueDepth = 2,
                                            outwardQueueDepth = 2,
                                            dataWidth_arg = 256), 
                                            sim = true))) ++
  new tlt.TLTConfig(maxInflight=1, beatBytes=32)
)


