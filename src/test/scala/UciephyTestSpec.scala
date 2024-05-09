package uciephytest

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec

class UciephyTestHarness(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Module {
  val io = IO(new UciephyTestMMIO)
  val test = Module(new UciephyTest(bufferDepthPerLane, numLanes))
  io <> test.io.mmio
  val phy = Module(new uciephytest.phy.Phy(numLanes))
  test.io.phy <> phy.io.test
  phy.io.top.refClock := clock
  phy.io.top.rxData := phy.io.top.txData
  phy.io.top.rxValid := phy.io.top.txValid
  phy.io.driverPuCtl := 0.U.asTypeOf(phy.io.driverPuCtl)
  phy.io.driverPdCtl := 0.U.asTypeOf(phy.io.driverPdCtl)
  phy.io.driverEn := 0.U.asTypeOf(phy.io.driverEn)
  phy.io.phaseCtl := 0.U.asTypeOf(phy.io.phaseCtl)
}

class UciephyTestSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UCIe PHY tester"
  it should "work" in {
    test(new UciephyTestHarness).withAnnotations(Seq(VcsBackendAnnotation, WriteFsdbAnnotation)) { c =>
      c.clock.setTimeout(1000)
      // Set up chip
      c.reset.poke(true.B)
      // Set up TX
      c.io.txDataChunkIn.initSource()
      c.io.txDataChunkIn.setSourceClock(c.clock)
      c.io.txValidFramingMode.poke(TxValidFramingMode.ucie)
      c.io.txBitsToSend.poke(64.U)
      c.io.txFsmRst.poke(true.B)
      // Set up RX
      c.io.rxValidStartThreshold.poke(4.U)
      c.io.rxFsmRst.poke(true.B)

      // Strobe reset
      for (i <- 0 until 64) {
        c.clock.step()
      }

      // Check reset state
      c.reset.poke(false.B)
      c.io.txFsmRst.poke(false.B)
      c.io.txTestState.expect(TxTestState.idle)
      c.io.txBitsSent.expect(0.U)
      c.io.rxBitsReceived.expect(0.U)
      c.io.rxFsmRst.poke(false.B)

      // Test TX data entry
      c.io.txDataLane.poke(0.U)
      c.io.txDataOffset.poke(0.U)
      c.io.txDataChunkIn.enqueueNow("h1234_5678_9abc_def0".U)
      c.clock.step()
      c.io.txDataChunkOut.expect("h1234_5678_9abc_def0".U)
      c.io.txDataLane.poke(1.U)
      c.io.txDataChunkIn.enqueueNow("h0fed_cba9_8765_4321".U)
      c.clock.step()
      c.io.txDataChunkOut.expect("h0fed_cba9_8765_4321".U)
      c.io.txTestState.expect(TxTestState.idle)

      // Start transmitting data
      c.io.txExecute.poke(true.B)
      c.clock.step()
      c.io.txExecute.poke(false.B)
      c.io.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.rxBitsReceived.peek().litValue < 64) {
        c.clock.step()
      }
      c.io.txTestState.expect(TxTestState.done)
      c.io.txBitsSent.expect(64.U)

      // Validate received data
      c.io.rxDataLane.poke(0.U)
      c.io.rxDataOffset.poke(0.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h1234_5678_9abc_def0".U)
      c.io.rxValidChunk.expect("h0f0f_0f0f_0f0f_0f0f".U)
      c.io.rxDataLane.poke(1.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h0fed_cba9_8765_4321".U)
      c.io.rxValidChunk.expect("h0f0f_0f0f_0f0f_0f0f".U)

      // Try a second transmission with different parameters.
      c.io.txValidFramingMode.poke(TxValidFramingMode.simple)
      c.io.txBitsToSend.poke(96.U)
      c.io.txFsmRst.poke(true.B)
      c.clock.step()
      c.io.txFsmRst.poke(false.B)
      c.io.txTestState.expect(TxTestState.idle)
      c.io.txBitsSent.expect(0.U)

      // Test TX data entry
      c.io.txDataLane.poke(0.U)
      c.io.txDataOffset.poke(0.U)
      c.io.txDataChunkIn.enqueueNow("h1234_5678_9abc_def0".U)
      c.clock.step()
      c.io.txDataOffset.poke(1.U)
      c.io.txDataChunkIn.enqueueNow("hdead_beef_cafe_f00d".U)
      c.clock.step()
      c.io.txDataLane.poke(1.U)
      c.io.txDataOffset.poke(0.U)
      c.io.txDataChunkIn.enqueueNow("h0fed_cba9_8765_4321".U)
      c.clock.step()
      c.io.txDataOffset.poke(1.U)
      c.io.txDataChunkIn.enqueueNow("hd00d_bead_deed_dea1".U)
      c.clock.step()
      c.io.txTestState.expect(TxTestState.idle)

      // Set up RX
      c.io.rxValidStartThreshold.poke(1.U)
      c.io.rxFsmRst.poke(true.B)
      c.clock.step()
      c.io.rxBitsReceived.expect(0.U)
      c.io.rxFsmRst.poke(false.B)
      c.clock.step()

      // Start transmitting data
      c.io.txExecute.poke(true.B)
      c.clock.step()
      c.io.txExecute.poke(false.B)
      c.io.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.rxBitsReceived.peek().litValue < 96) {
        c.clock.step()
      }
      c.io.txTestState.expect(TxTestState.done)
      c.io.txBitsSent.expect(96.U)

      // Validate received data
      c.io.rxDataLane.poke(0.U)
      c.io.rxDataOffset.poke(0.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h1234_5678_9abc_def0".U)
      c.io.rxValidChunk.expect("hffff_ffff_ffff_ffff".U)
      c.io.rxDataOffset.poke(1.U)
      c.clock.step()
      assert((c.io.rxDataChunk.peek().litValue & BigInt(0xffffffff)) == BigInt(0xcafef00d))
      c.io.rxValidChunk.expect("h0000_0000_ffff_ffff".U)
      c.io.rxDataLane.poke(1.U)
      c.io.rxDataOffset.poke(0.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h0fed_cba9_8765_4321".U)
      c.io.rxValidChunk.expect("hffff_ffff_ffff_ffff".U)
      c.io.rxDataOffset.poke(1.U)
      c.clock.step()
      assert((c.io.rxDataChunk.peek().litValue & BigInt(0xffffffff)) == BigInt(0xdeeddea1))
      c.io.rxValidChunk.expect("h0000_0000_ffff_ffff".U)
    }
  }
}
