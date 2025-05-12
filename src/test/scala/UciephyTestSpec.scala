package uciephytest

import chisel3._
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import chisel3.experimental.VecLiterals._

class UciephyTestHarness(bufferDepthPerLane: Int = 10, numLanes: Int = 2)
    extends Module {
  val io = IO(new Bundle {
    val mmio = new UciephyTestMMIO
    val shufflerCtl = Input(Vec(numLanes + 4, Vec(32, UInt(5.W))))
  })
  val test = Module(new UciephyTest(bufferDepthPerLane, numLanes))
  io.mmio <> test.io.mmio
  val phy = Module(new uciephytest.phy.Phy(numLanes, sim = true))
  test.io.phy <> phy.io.test
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
  phy.io.txctl := DontCare
  phy.io.rxctl := DontCare
  phy.io.pllCtl := DontCare
  phy.io.testPllCtl := DontCare
  phy.io.pllBypassEn := false.B
  for (i <- 0 until numLanes + 4) {
    phy.io.txctl(i).shuffler := io.shufflerCtl(i)
  }
}

class UciephyTestSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UCIe PHY tester"
  it should "work in manual test mode" in {
    test(new UciephyTestHarness)
      .withAnnotations(Seq(VcsBackendAnnotation, WriteVcdAnnotation)) { c =>
        c.clock.setTimeout(1000)
        // Set up chip
        c.reset.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        c.reset.poke(false.B)

        // Set up TX
        c.io.mmio.txClkP.poke("h0f0f0f0f".U)
        c.io.mmio.txDataChunkIn.initSource()
        c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
        c.io.shufflerCtl.poke(
          Vec.Lit(
            Seq.fill(6)(
              Vec.Lit((0 until 32).map(i => i.U(5.W)): _*)
            ): _*
          )
        )
        c.io.mmio.txDataMode.poke(DataMode.finite)
        c.io.mmio.txPacketsToSend.poke(2.U)
        c.io.mmio.txFsmRst.poke(true.B)
        // Set up RX
        c.io.mmio.rxDataMode.poke(DataMode.infinite)
        c.io.mmio.rxFsmRst.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        // Check reset state
        c.io.mmio.txFsmRst.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.idle)
        c.io.mmio.txPacketsSent.expect(0.U)
        c.io.mmio.rxPacketsReceived.expect(0.U)
        c.io.mmio.rxFsmRst.poke(false.B)

        // Test TX data entry
        c.io.mmio.txDataLaneGroup.poke(0.U)
        c.io.mmio.txDataOffset.poke(0.U)
        c.io.mmio.txDataChunkIn.enqueueNow("h0f0f_0f0f_8765_4321_9abc_def0".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataChunkOut.expect("h0f0f_0f0f_8765_4321_9abc_def0".U)
        c.io.mmio.txDataOffset.poke(1.U)
        c.io.mmio.txDataChunkIn.enqueueNow("h0f0f_0f0f_0fed_cba9_1234_5678".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataChunkOut.expect("h0f0f_0f0f_0fed_cba9_1234_5678".U)
        c.io.mmio.txTestState.expect(TxTestState.idle)

        // Give time for clock to reach receiver.
        for (i <- 0 until 100) {
          c.clock.step()
        }

        // Start transmitting data
        c.io.mmio.txExecute.poke(true.B)
        c.clock.step()
        c.io.mmio.txExecute.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.run)

        // Wait until all bits are received
        while (c.io.mmio.rxPacketsReceived.peek().litValue < 2) {
          c.clock.step()
        }
        c.io.mmio.txTestState.expect(TxTestState.done)
        c.io.mmio.txPacketsSent.expect(2.U)

        // Validate received data
        c.io.mmio.rxDataLane.poke(0.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h9abc_def0".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h1234_5678".U)
        c.io.mmio.rxDataLane.poke(1.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h8765_4321".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0fed_cba9".U)
        c.io.mmio.rxDataLane.poke(2.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0f0f_0f0f".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0f0f_0f0f".U)

        // Try a second transmission with different parameters.
        c.io.mmio.txPacketsToSend.poke(3.U)
        c.io.mmio.txFsmRst.poke(true.B)
        c.clock.step()
        c.io.mmio.txFsmRst.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.idle)
        c.io.mmio.txPacketsSent.expect(0.U)

        // Test TX data entry
        c.io.mmio.txDataLaneGroup.poke(0.U)
        c.io.mmio.txDataOffset.poke(0.U)
        c.io.mmio.txDataChunkIn.enqueueNow("hffff_ffff_8765_4321_9abc_def0".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataOffset.poke(1.U)
        c.io.mmio.txDataChunkIn.enqueueNow("hffff_ffff_0fed_cba9_1234_5678".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataOffset.poke(2.U)
        c.io.mmio.txDataChunkIn.enqueueNow("hffff_ffff_deed_dea1_cafe_f00d".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txTestState.expect(TxTestState.idle)

        // Set up RX
        c.io.mmio.rxDataMode.poke(DataMode.infinite)
        c.io.mmio.rxFsmRst.poke(true.B)
        c.clock.step()
        c.io.mmio.rxPacketsReceived.expect(0.U)
        c.io.mmio.rxFsmRst.poke(false.B)
        c.clock.step()

        // Start transmitting data
        c.io.mmio.txExecute.poke(true.B)
        c.clock.step()
        c.io.mmio.txExecute.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.run)

        // Wait until all bits are received
        while (c.io.mmio.rxPacketsReceived.peek().litValue < 3) {
          c.clock.step()
        }
        c.io.mmio.txTestState.expect(TxTestState.done)
        c.io.mmio.txPacketsSent.expect(3.U)

        // Validate received data
        c.io.mmio.rxDataLane.poke(0.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h9abc_def0".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h1234_5678".U)
        c.io.mmio.rxDataOffset.poke(2.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("hcafef00d".U)
        c.io.mmio.rxDataLane.poke(1.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h8765_4321".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0fed_cba9".U)
        c.io.mmio.rxDataOffset.poke(2.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("hdeeddea1".U)
        c.io.mmio.rxDataLane.poke(2.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("hffff_ffff".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("hffff_ffff".U)
        c.io.mmio.rxDataOffset.poke(2.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("hffff_ffff".U)
      }
  }

  it should "work in lfsr test mode" in {
    test(new UciephyTestHarness)
      .withAnnotations(Seq(VcsBackendAnnotation, WriteVcdAnnotation)) { c =>
        c.clock.setTimeout(10000)
        // Set up chip
        c.reset.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        c.reset.poke(false.B)

        // Set up TX
        c.io.mmio.txClkP.poke("h0f0f0f0f".U)
        c.io.mmio.txDataChunkIn.initSource()
        c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
        c.io.shufflerCtl.poke(
          Vec.Lit(
            Seq.fill(6)(
              Vec.Lit((0 until 32).map(i => i.U(5.W)): _*)
            ): _*
          )
        )
        c.io.mmio.txTestMode.poke(TxTestMode.lfsr)
        c.io.mmio.txDataMode.poke(DataMode.finite)
        c.io.mmio.txPacketsToSend.poke(8.U)
        c.io.mmio.txFsmRst.poke(true.B)
        // Set up RX
        c.io.mmio.rxDataMode.poke(DataMode.infinite)
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
        c.io.mmio.txPacketsSent.expect(0.U)
        c.io.mmio.rxPacketsReceived.expect(0.U)
        c.io.mmio.rxFsmRst.poke(false.B)

        // Give time for clock to reach receiver.
        for (i <- 0 until 100) {
          c.clock.step()
        }

        // Start transmitting data
        c.io.mmio.txExecute.poke(true.B)
        c.clock.step()
        c.io.mmio.txExecute.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.run)

        // Wait until 256 bits are received
        while (c.io.mmio.rxPacketsReceived.peek().litValue < 8) {
          c.clock.step()
        }

        // Validate no bit errors
        for (lane <- 0 until 2) {
          c.io.mmio.rxBitErrors(lane).expect(0.U)
        }

        // Try a second transmission where the LFSR seeds to not match up
        c.io.mmio.txFsmRst.poke(true.B)

        // Set seeds.
        c.io.mmio.txLfsrSeed(0).poke(3.U)
        c.io.mmio.rxLfsrSeed(0).poke(3.U)
        c.io.mmio.txLfsrSeed(1).poke(1.U)
        c.io.mmio.rxLfsrSeed(1).poke(3.U)

        // Set up RX
        c.io.mmio.rxDataMode.poke(DataMode.infinite)
        c.io.mmio.rxFsmRst.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        c.io.mmio.txFsmRst.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.idle)
        c.io.mmio.txPacketsSent.expect(0.U)
        c.io.mmio.rxPacketsReceived.expect(0.U)
        c.io.mmio.rxFsmRst.poke(false.B)
        c.clock.step()

        // Give time for clock to reach receiver.
        for (i <- 0 until 100) {
          c.clock.step()
        }

        // Start transmitting data
        c.io.mmio.txExecute.poke(true.B)
        c.clock.step()
        c.io.mmio.txExecute.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.run)

        // Wait until all bits are received
        while (c.io.mmio.rxPacketsReceived.peek().litValue < 8) {
          c.clock.step()
        }
        c.io.mmio.rxBitErrors(0).expect(0.U)
        assert(c.io.mmio.rxBitErrors(1).peek().litValue > 0)
      }
  }

  it should "support configurable data shuffling" in {
    test(new UciephyTestHarness)
      .withAnnotations(Seq(VcsBackendAnnotation, WriteVcdAnnotation)) { c =>
        c.clock.setTimeout(1000)
        // Set up chip
        c.reset.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        c.reset.poke(false.B)

        // Set up TX
        c.io.mmio.txClkP.poke("h0f0f0f0f".U)
        c.io.mmio.txDataChunkIn.initSource()
        c.io.shufflerCtl.poke(
          Vec.Lit(
            (0 until 6).map(lane =>
              if (lane < 2) {
                Vec.Lit((0 until 32).map(i => (31 - i).U(5.W)): _*)
              } else {
                Vec.Lit((0 until 32).map(i => i.U(5.W)): _*)
              }
            ): _*
          )
        )
        c.io.mmio.txDataChunkIn.setSourceClock(c.clock)
        c.io.mmio.txPacketsToSend.poke(2.U)
        c.io.mmio.txFsmRst.poke(true.B)
        // Set up RX
        c.io.mmio.rxDataMode.poke(DataMode.infinite)
        c.io.mmio.rxFsmRst.poke(true.B)

        // Strobe reset
        for (i <- 0 until 64) {
          c.clock.step()
        }

        // Check reset state
        c.io.mmio.txFsmRst.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.idle)
        c.io.mmio.txPacketsSent.expect(0.U)
        c.io.mmio.rxPacketsReceived.expect(0.U)
        c.io.mmio.rxFsmRst.poke(false.B)

        // Test TX data entry
        c.io.mmio.txDataLaneGroup.poke(0.U)
        c.io.mmio.txDataOffset.poke(0.U)
        c.io.mmio.txDataChunkIn.enqueueNow("h0f0f_0f0f_8765_4321_9abc_def0".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataChunkOut.expect("h0f0f_0f0f_8765_4321_9abc_def0".U)
        c.io.mmio.txDataOffset.poke(1.U)
        c.io.mmio.txDataChunkIn.enqueueNow("h0f0f_0f0f_0fed_cba9_1234_5678".U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.txDataChunkOut.expect("h0f0f_0f0f_0fed_cba9_1234_5678".U)
        c.io.mmio.txTestState.expect(TxTestState.idle)

        // Give time for clock to reach receiver.
        for (i <- 0 until 100) {
          c.clock.step()
        }

        // Start transmitting data
        c.io.mmio.txExecute.poke(true.B)
        c.clock.step()
        c.io.mmio.txExecute.poke(false.B)
        c.io.mmio.txTestState.expect(TxTestState.run)

        // Wait until all bits are received
        while (c.io.mmio.rxPacketsReceived.peek().litValue < 2) {
          c.clock.step()
        }
        c.io.mmio.txTestState.expect(TxTestState.done)
        c.io.mmio.txPacketsSent.expect(2.U)

        // Validate received data
        c.io.mmio.rxDataLane.poke(0.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0f7b_3d59".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h1e6a_2c48".U)
        c.io.mmio.rxDataLane.poke(1.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h84c2_a6e1".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h95d3_b7f0".U)
        c.io.mmio.rxDataLane.poke(2.U)
        c.io.mmio.rxDataOffset.poke(0.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0f0f_0f0f".U)
        c.io.mmio.rxDataOffset.poke(1.U)
        for (i <- 0 until 4) {
          c.clock.step()
        }
        c.io.mmio.rxDataChunk.expect("h0f0f_0f0f".U)
      }
  }
}
