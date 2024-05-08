package uciephytest

import chisel3._
import chisel3.experimental.VecLiterals.AddObjectLiteralConstructor
import chiseltest._
import freechips.rocketchip.util.AsyncQueueParams
import org.scalatest.flatspec.AnyFlatSpec
import interfaces._

class UciephyTestSpec extends AnyFlatSpec with ChiselScalatestTester {
  behavior of "UCIe PHY tester"
  it should "work" in {
    test(new UciephyTest(bufferDepthPerLane = 10, numLanes = 2)) { c =>
      // Set up TX
      c.io.txDataChunkIn.initSource()
      c.io.txData.setSourceClock(c.clock)
      c.io.txValidFramingMode.poke(TxValidFramingMode.ucie)
      c.io.txFsmRst.poke(true.B)
      c.io.txBitsToSend.poke(64.U)
      c.clock.step()
      c.io.txFsmRst.poke(false.B)
      c.io.txTestState.expect(TxTestState.idle)
      c.io.txBitsSent.expect(0.U)

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

      // Set up RX
      c.io.rxFsmRst.poke(true.B)
      c.clock.step()
      c.io.rxFsmRst.poke(false.B)
      c.io.rxBitsReceived.expect(0.U)
      c.io.rxValidStartThreshold.poke(4.U)
      c.clock.step()

      // Start transmitting data
      c.io.txExecute.poke(true.B)
      c.clock.step()
      c.io.txExecute.poke(false.B)
      c.io.txTestState.expect(TxTestState.run)

      // Wait until all bits are received
      while (c.io.rxBitsReceived.peek() < 64) {
        c.clock.step()
      }
      c.io.rxBitsReceived.expect(64.U)
      c.io.txTestState.expect(TxTestState.done)
      c.io.txBitsSent.expect(64.U)

      // Validate received data
      c.io.rxDataLane.poke(0.U)
      c.io.rxDataOffset.poke(0.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h1234_5678_9abc_def0".U)
      c.io.rxValidChunk.expect("h1010_1010_1010_1010".U)
      c.io.rxDataLane.poke(1.U)
      c.clock.step()
      c.io.rxDataChunk.expect("h0fed_cba9_8765_4321".U)
      c.io.rxValidChunk.expect("h1010_1010_1010_1010".U)




      c.io.mainbandLaneIO.txData.enqueueNow(
        "h1234_5678_9abc_def0_0fed_cba9_8765_4321_1111_2222_3333_4444_5555_6666_7777_8888".U,
      )
      c.io.mainbandIo.txData
        .expectDequeueNow(
          Vec.Lit(
            "h1211".U,
            "h3411".U,
            "h5622".U,
            "h7822".U,
            "h9a33".U,
            "hbc33".U,
            "hde44".U,
            "hf044".U,
            "h0f55".U,
            "hed55".U,
            "hcb66".U,
            "ha966".U,
            "h8777".U,
            "h6577".U,
            "h4388".U,
            "h2188".U,
          ),
        )
    }
  }
}
