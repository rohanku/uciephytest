package uciephytest

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import chisel3.experimental.VecLiterals._

class UciephyTestHarness(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Module {
  val io = IO(new Bundle {
    val mmio = new UciephyTestMMIO
    val shufflerCtl = Input(Vec(numLanes + 1, Vec(16, UInt(4.W))))
  })
  val test = Module(new UciephyTest(bufferDepthPerLane, numLanes))
  io.mmio <> test.io.mmio
  val phy = Module(new uciephytest.phy.Phy(numLanes, sim = true))
  test.io.phy <> phy.io.test
  phy.io.top.refClkP := clock
  phy.io.top.refClkN := (!clock.asBool).asClock
  phy.io.top.rxClkP := phy.io.top.txClkP
  phy.io.top.rxClkN := phy.io.top.txClkN
  phy.io.top.rxData := phy.io.top.txData
  phy.io.top.rxValid := phy.io.top.txValid
  phy.io.top.pllIref := false.B
  phy.io.driverPuCtl := 0.U.asTypeOf(phy.io.driverPuCtl)
  phy.io.driverPdCtl := 0.U.asTypeOf(phy.io.driverPdCtl)
  phy.io.driverEn := 0.U.asTypeOf(phy.io.driverEn)
  phy.io.clockingMiscCtl := 0.U.asTypeOf(phy.io.clockingMiscCtl)
  phy.io.clockingPiCtl := 0.U.asTypeOf(phy.io.clockingPiCtl)
  phy.io.terminationCtl := 0.U.asTypeOf(phy.io.terminationCtl)
  phy.io.vrefCtl := 0.U.asTypeOf(phy.io.vrefCtl)
  phy.io.shufflerCtl := io.shufflerCtl
}

class UciephyTestSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UCIe PHY tester"
  it should "work in manual test mode" in {
    test(new UciephyTestHarness).withAnnotations(Seq(VcsBackendAnnotation, WriteFsdbAnnotation)) { c =>
      c.clock.setTimeout(1000)
      // Set up chip
      c.reset.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      c.reset.poke(false.B)

      // Set up TX
      c.io.mmio.txDataChunkIn.initSource()
      c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
      c.io.shufflerCtl.poke(Vec.Lit(Seq.fill(3)(
        Vec.Lit((0 until 16).map(i => i.U(4.W)):_*)
      ):_*))
      c.io.mmio.txValidFramingMode.poke(TxValidFramingMode.ucie)
      c.io.mmio.txBitsToSend.poke(64.U)
      c.io.mmio.txFsmRst.poke(true.B)
      // Set up RX
      c.io.mmio.rxValidStartThreshold.poke(4.U)
      c.io.mmio.rxFsmRst.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      // Check reset state
      c.io.mmio.txFsmRst.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.idle)
      c.io.mmio.txBitsSent.expect(0.U)
      c.io.mmio.rxBitsReceived.expect(0.U)
      c.io.mmio.rxFsmRst.poke(false.B)

      // Test TX data entry
      c.io.mmio.txDataLane.poke(0.U)
      c.io.mmio.txDataOffset.poke(0.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h1234_5678_9abc_def0".U)
      c.clock.step()
      c.io.mmio.txDataChunkOut.expect("h1234_5678_9abc_def0".U)
      c.io.mmio.txDataLane.poke(1.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h0fed_cba9_8765_4321".U)
      c.clock.step()
      c.io.mmio.txDataChunkOut.expect("h0fed_cba9_8765_4321".U)
      c.io.mmio.txTestState.expect(TxTestState.idle)

      // Start transmitting data
      c.io.mmio.txExecute.poke(true.B)
      c.clock.step()
      c.io.mmio.txExecute.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.mmio.rxBitsReceived.peek().litValue < 64) {
        c.clock.step()
      }
      c.io.mmio.txTestState.expect(TxTestState.done)
      c.io.mmio.txBitsSent.expect(64.U)

      // Validate received data
      c.io.mmio.rxDataLane.poke(0.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h9abc_def0".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h1234_5678".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataLane.poke(1.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h8765_4321".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h0fed_cba9".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)

      // Try a second transmission with different parameters.
      c.io.mmio.txValidFramingMode.poke(TxValidFramingMode.simple)
      c.io.mmio.txBitsToSend.poke(96.U)
      c.io.mmio.txFsmRst.poke(true.B)
      c.clock.step()
      c.io.mmio.txFsmRst.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.idle)
      c.io.mmio.txBitsSent.expect(0.U)

      // Test TX data entry
      c.io.mmio.txDataLane.poke(0.U)
      c.io.mmio.txDataOffset.poke(0.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h1234_5678_9abc_def0".U)
      c.clock.step()
      c.io.mmio.txDataOffset.poke(1.U)
      c.io.mmio.txDataChunkIn.enqueueNow("hdead_beef_cafe_f00d".U)
      c.clock.step()
      c.io.mmio.txDataLane.poke(1.U)
      c.io.mmio.txDataOffset.poke(0.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h0fed_cba9_8765_4321".U)
      c.clock.step()
      c.io.mmio.txDataOffset.poke(1.U)
      c.io.mmio.txDataChunkIn.enqueueNow("hd00d_bead_deed_dea1".U)
      c.clock.step()
      c.io.mmio.txTestState.expect(TxTestState.idle)

      // Set up RX
      c.io.mmio.rxValidStartThreshold.poke(31.U)
      c.io.mmio.rxFsmRst.poke(true.B)
      c.clock.step()
      c.io.mmio.rxBitsReceived.expect(0.U)
      c.io.mmio.rxFsmRst.poke(false.B)
      c.clock.step()

      // Start transmitting data
      c.io.mmio.txExecute.poke(true.B)
      c.clock.step()
      c.io.mmio.txExecute.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.mmio.rxBitsReceived.peek().litValue < 96) {
        c.clock.step()
      }
      c.io.mmio.txTestState.expect(TxTestState.done)
      c.io.mmio.txBitsSent.expect(96.U)

      // Validate received data
      c.io.mmio.rxDataLane.poke(0.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h9abc_def0".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h1234_5678".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
      c.io.mmio.rxDataOffset.poke(2.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("hcafef00d".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
      c.io.mmio.rxDataLane.poke(1.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h8765_4321".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h0fed_cba9".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
      c.io.mmio.rxDataOffset.poke(2.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("hdeeddea1".U)
      c.io.mmio.rxValidChunk.expect("hffff_ffff".U)
    }
  }

  it should "work in lfsr test mode" in {
    test(new UciephyTestHarness).withAnnotations(Seq(VcsBackendAnnotation, WriteFsdbAnnotation)) { c =>
      c.clock.setTimeout(1000)
      // Set up chip
      c.reset.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      c.reset.poke(false.B)

      // Set up TX
      c.io.mmio.txDataChunkIn.initSource()
      c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
      c.io.shufflerCtl.poke(Vec.Lit(Seq.fill(3)(
        Vec.Lit((0 until 16).map(i => i.U(4.W)):_*)
      ):_*))
      c.io.mmio.txTestMode.poke(TxTestMode.lfsr)
      c.io.mmio.txValidFramingMode.poke(TxValidFramingMode.ucie)
      c.io.mmio.txFsmRst.poke(true.B)
      // Set up RX
      c.io.mmio.rxValidStartThreshold.poke(4.U)
      c.io.mmio.rxFsmRst.poke(true.B)

      // Set seeds.
      for (lane <- 0 until 2) {
        c.io.mmio.txLfsrSeed(lane).poke(1.U)
        c.io.mmio.rxLfsrSeed(lane).poke(1.U)
      }

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      // Check reset state
      c.io.mmio.txFsmRst.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.idle)
      c.io.mmio.txBitsSent.expect(0.U)
      c.io.mmio.rxBitsReceived.expect(0.U)
      c.io.mmio.rxFsmRst.poke(false.B)

      // Start transmitting data
      c.io.mmio.txExecute.poke(true.B)
      c.clock.step()
      c.io.mmio.txExecute.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.run)

      // Wait until 256 bits are received
      while (c.io.mmio.rxBitsReceived.peek().litValue < 256) {
        c.clock.step()
      }

      // Validate no bit errors
      for (lane <- 0 until 2) {
        c.io.mmio.rxBitErrors(lane).expect(0.U)
      }

      // Try a second transmission where the LFSR seeds to not match up
      c.io.mmio.txValidFramingMode.poke(TxValidFramingMode.simple)
      c.io.mmio.txFsmRst.poke(true.B)

      // Set seeds.
      c.io.mmio.txLfsrSeed(0).poke(3.U)
      c.io.mmio.rxLfsrSeed(0).poke(3.U)
      c.io.mmio.txLfsrSeed(1).poke(1.U)
      c.io.mmio.rxLfsrSeed(1).poke(3.U)

      c.clock.step()

      c.io.mmio.txFsmRst.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.idle)
      c.io.mmio.txBitsSent.expect(0.U)

      // Set up RX
      c.io.mmio.rxValidStartThreshold.poke(31.U)
      c.io.mmio.rxFsmRst.poke(true.B)
      c.clock.step()
      c.io.mmio.rxBitsReceived.expect(0.U)
      c.io.mmio.rxFsmRst.poke(false.B)
      c.clock.step()

      // Start transmitting data
      c.io.mmio.txExecute.poke(true.B)
      c.clock.step()
      c.io.mmio.txExecute.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.mmio.rxBitsReceived.peek().litValue < 256) {
        c.clock.step()
      }
      c.io.mmio.rxBitErrors(0).expect(0.U)
      assert(c.io.mmio.rxBitErrors(1).peek().litValue > 0)
    }
  }

  it should "support configurable data shuffling" in {
    test(new UciephyTestHarness).withAnnotations(Seq(VcsBackendAnnotation, WriteFsdbAnnotation)) { c =>
      c.clock.setTimeout(1000)
      // Set up chip
      c.reset.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      c.reset.poke(false.B)

      // Set up TX
      c.io.mmio.txDataChunkIn.initSource()
      c.io.shufflerCtl.poke(Vec.Lit((0 until 3).map(lane =>
        if (lane < 2) {
          Vec.Lit((0 until 16).map(i => (15 - i).U(4.W)):_*)
        } else {
          Vec.Lit((0 until 16).map(i => i.U(4.W)):_*)
        }
      ):_*))
      c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
      c.io.mmio.txValidFramingMode.poke(TxValidFramingMode.ucie)
      c.io.mmio.txBitsToSend.poke(64.U)
      c.io.mmio.txFsmRst.poke(true.B)
      // Set up RX
      c.io.mmio.rxValidStartThreshold.poke(4.U)
      c.io.mmio.rxFsmRst.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      // Check reset state
      c.io.mmio.txFsmRst.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.idle)
      c.io.mmio.txBitsSent.expect(0.U)
      c.io.mmio.rxBitsReceived.expect(0.U)
      c.io.mmio.rxFsmRst.poke(false.B)

      // Test TX data entry
      c.io.mmio.txDataLane.poke(0.U)
      c.io.mmio.txDataOffset.poke(0.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h1234_5678_9abc_def0".U)
      c.clock.step()
      c.io.mmio.txDataChunkOut.expect("h1234_5678_9abc_def0".U)
      c.io.mmio.txDataLane.poke(1.U)
      c.io.mmio.txDataChunkIn.enqueueNow("h0fed_cba9_8765_4321".U)
      c.clock.step()
      c.io.mmio.txDataChunkOut.expect("h0fed_cba9_8765_4321".U)
      c.io.mmio.txTestState.expect(TxTestState.idle)

      // Start transmitting data
      c.io.mmio.txExecute.poke(true.B)
      c.clock.step()
      c.io.mmio.txExecute.poke(false.B)
      c.io.mmio.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.mmio.rxBitsReceived.peek().litValue < 64) {
        c.clock.step()
      }
      c.io.mmio.txTestState.expect(TxTestState.done)
      c.io.mmio.txBitsSent.expect(64.U)

      // Validate received data
      c.io.mmio.rxDataLane.poke(0.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h3d59_0f7b".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("h2c48_1e6a".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataLane.poke(1.U)
      c.io.mmio.rxDataOffset.poke(0.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("ha6e1_84c2".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
      c.io.mmio.rxDataOffset.poke(1.U)
      for (i <- 0 until 4) {
        c.clock.step()
      }
      c.io.mmio.rxDataChunk.expect("hb7f0_95d3".U)
      c.io.mmio.rxValidChunk.expect("h0f0f_0f0f".U)
    }
  }
}
