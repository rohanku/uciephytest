package uciephytest

import chisel3._
import chisel3.util._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField, RegWriteFn}
import freechips.rocketchip.tilelink._
import uciephytest.phy.{Phy, PhyToTestIO}

case class UciephyTestParams(
  address: BigInt = 0x4000,
  bufferDepthPerLane: Int = 10,
  numLanes: Int = 2,
)

case object UciephyTestKey extends Field[Option[UciephyTestParams]](None)


// TX test modes.
object TxTestMode extends ChiselEnum {
  // Data to send is provided manually via `txDataOffset` and `txDataChunkIn`.
  val manual = Value(0.U(1.W))
  // Data to send is derived from an LFSR.
  val lfsr = Value(1.U(1.W))
}

// TX valid framing modes.
object TxValidFramingMode extends ChiselEnum {
  // Valid framing is done according to the UCIe spec (high for 4 UI, low for 4 UI).
  val ucie = Value(0.U(1.W))
  // Valid is high while valid data is being transmitted, and low when data is no longer valid.
  val simple = Value(1.U(1.W))
}

// State of the TX test FSM.
object TxTestState extends ChiselEnum {
  // Awaiting configuration and start of transmission.
  val idle = Value(0.U(2.W))
  // Test is currently being run.
  val run = Value(1.U(2.W))
  // Test is complete.
  val done = Value(2.U(2.W))
}

class UciephyTopIO(numLanes: Int = 4) extends Bundle {
  val txData = Output(Vec(numLanes, Bool()))
  val txValid = Output(Bool())
  val refClock = Input(Clock())
  // val txClkP = Output(Clock())
  // val txClkN = Output(Clock())
  val rxData = Input(Vec(numLanes, Bool()))
  val rxValid = Input(Bool())
  // val rxClkP = Input(Clock())
  // val rxClkN = Input(Clock())
}

class UciephyTestIO(bufferDepthPerLane: Int = 10, numLanes: Int = 4) extends Bundle {
  // TODO: Add any required clocking control signals.

  /// TX CONTROL
  // =====================
  // The test mode of the TX.
  val txTestMode = Input(TxTestMode())
  // The valid framing mode of the TX.
  val txValidFramingMode = Input(TxValidFramingMode())
  // Seed of the TX LFSR.
  val txLfsrSeed = Input(Vec(numLanes, UInt(Phy.SerdesRatio.W)))
  // Repeats the TX manual transmission indefinitely. Useful for sending a long custom pattern
  // and verifying the `rxSignature`.
  val txManualRepeat = Input(Bool())
  // Resets the TX FSM (i.e. resetting the number of bits sent to 0, reseeding the LFSR,
  // and stopping any in-progress transmissions).
  val txFsmRst = Input(Bool())
  // Starts a transmission starting from the beginning of the input buffer (`TxTestMode.manual`) or from
  // the current state of the LFSR (`TxTestMode.lsfr`). Does not do anything if a transmission is in progress.
  val txExecute = Input(Bool())
  // The number of bits sent since the last FSM reset.
  val txBitsSent = Output(UInt(bufferDepthPerLane.W))
  // The number of bits to send during transmission. Set to 0 to send the entire buffer (`TxTestMode.manual`)
  // or continuously (`TxTestMode.lsfr`). Numbers greater than the buffer length will send the entire buffer
  // in `TxTestMode.manual`.
  val txBitsToSend = Input(UInt(32.W))
  // Data chunk lane in input buffer.
  val txDataLane = Input(UInt(log2Ceil(numLanes).W))
  // Data chunk offset in input buffer.
  val txDataOffset = Input(UInt((bufferDepthPerLane - 6).W))
  // 64-bit data chunk to write.
  val txDataChunkIn = Flipped(DecoupledIO(UInt(64.W)))
  // Data chunk at the given chunk offset for inspecting the data to be sent.
  val txDataChunkOut = Output(UInt(64.W))
  // Permutation of data fed into the serializer, in case serializer timing is off.
  val txPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))
  // State of the TX test FSM.
  val txTestState = Output(TxTestState())

  // RX CONTROL
  // ====================
  // Seed of the RX LFSR used for detecting bit errors. Should be the same as the TX seed of the transmitting chiplet.
  val rxLfsrSeed = Input(Vec(numLanes, UInt(Phy.SerdesRatio.W)))
  // The number of bit periods that valid must be high for the transmission to be considered valid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 1 such that data is received as long
  // as valid goes high for at least 1 bit period.
  val rxValidStartThreshold = Input(UInt(4.W))
  // The number of bit periods that valid must be low for the transmission to be considered invalid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 0 so that
  // data starts being received as soon as valid goes high and ignores any glitches afterwards.
  val rxValidStopThreshold = Input(UInt(4.W))
  // Resets the RX FSM (i.e. resetting the number of bits received and the offset within the output
  // buffer to 0).
  val rxFsmRst = Input(Bool())
  // The number of bits received since the last FSM reset. Only the first 2^bufferDepthPerLane bits received 
  // per lane are stored in the output buffer.
  val rxBitsReceived = Output(UInt(bufferDepthPerLane.W))
  // The number of bit errors since the last FSM reset. Only applicable in `TxTestMode.lsfr`.
  val rxBitErrors = Output(UInt(bufferDepthPerLane.W))
  // A MISR derived from the bits received since the last FSM reset.
  val rxSignature = Output(UInt(32.W))
  // Data chunk lane in output buffer.
  val rxDataLane = Input(UInt(log2Ceil(numLanes).W))
  // Data chunk offset in output buffer.
  val rxDataOffset = Input(UInt((bufferDepthPerLane - 6).W))
  // Data chunk at the given chunk offset for inspect the received data.
  val rxDataChunk = Output(UInt(64.W))
  // Valid chunk at the given chunk offset for inspect the valid signals corresponding to the received data.
  // In a correct UCIe implementation, there should be 4 1-bits followed by 4 0-bits repeated across the entire 
  // transmission.
  val rxValidChunk = Output(UInt(64.W))
  // Permutation of data read from the deserializer, in case deserializer timing is off.
  val rxPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))

  // PHY INTERFACE
  // ====================
  val phy = Flipped(new PhyToTestIO(numLanes))
}

class UciephyTest(bufferDepthPerLane: Int = 10, numLanes: Int = 4) extends Module {
  val io = IO(new UciephyTestIO(bufferDepthPerLane, numLanes))

  // TX registers
  val txState = RegInit(TxTestState.idle)
  val packetsEnqueued = RegInit(VecInit(Seq.fill(numLanes + 1)(0.U)))

  // RX registers
  val rxBitsReceived = RegInit(0.U)
  val rxBitErrors = RegInit(0.U)
  val rxSignature = RegInit(0.U)

  val totalChunks = numLanes * (2 << (bufferDepthPerLane - 6))
  val addrBits = log2Ceil(totalChunks)

  val inputBuffer = Reg(Vec(numLanes, Vec(2 << (bufferDepthPerLane - 6), UInt(64.W))))
  val outputDataBuffer = Reg(Vec(numLanes, Vec(2 << (bufferDepthPerLane - 6), UInt(64.W))))
  val outputValidBuffer = Reg(Vec(2 << (bufferDepthPerLane - 6), UInt(64.W)))
  val txDataAddr = Wire(UInt(addrBits.W))
  val rxDataAddr = Wire(UInt(addrBits.W))

  // Use valid lane as reference for how many bits were sent.
  io.txBitsSent := packetsEnqueued(numLanes) << log2Ceil(Phy.DigitalBitsPerCycle)
  io.txDataChunkIn.ready := txState === TxTestState.idle
  io.txDataChunkOut := inputBuffer(txDataAddr)
  io.txTestState := txState

  io.rxBitsReceived := rxBitsReceived
  io.rxBitErrors := rxBitErrors
  io.rxSignature := rxSignature
  io.rxDataChunk := outputDataBuffer(io.rxDataLane)(io.rxDataOffset)
  io.rxValidChunk := outputValidBuffer(io.rxDataLane)(io.rxDataOffset)

  for (lane <- 0 until numLanes) {
    io.phy.txTransmitData(lane).bits := 0.U
    io.phy.txTransmitData(lane).valid := false.B
    io.phy.rxReceiveData(lane).ready := false.B
  }
  io.phy.txTransmitValid.bits := 0.U
  io.phy.txTransmitValid.valid := false.B
  io.phy.rxReceiveValid.ready := false.B

  // TX logic
  switch(txState) {
    is(TxTestState.idle) {
      when (io.txDataChunkIn.valid) {
        inputBuffer(io.txDataLane)(io.txDataOffset) := io.txDataChunkIn.bits
      }

      when (io.txExecute) {
        txState := TxTestState.run
      }
    }
    is(TxTestState.run) {
      val dividedInputBuffer = inputBuffer.asTypeOf(Vec(numLanes, Vec(2<<bufferDepthPerLane / Phy.DigitalBitsPerCycle, UInt(Phy.DigitalBitsPerCycle.W))))
      for (lane <- 0 until numLanes) {
        io.phy.txTransmitData(lane).bits := dividedInputBuffer(lane)(packetsEnqueued(lane))
        io.phy.txTransmitData(lane).valid := packetsEnqueued(lane) << log2Ceil(Phy.DigitalBitsPerCycle) < io.txBitsToSend
      }
      switch(io.txValidFramingMode) {
        is (TxValidFramingMode.ucie) {
          io.phy.txTransmitValid.bits := VecInit((0 until Phy.DigitalBitsPerCycle/8).flatMap(_ => Seq.fill(4)(true.B) ++ Seq.fill(4)(false.B))).asUInt
        }
        is (TxValidFramingMode.simple) {
          io.phy.txTransmitValid.bits := VecInit(Seq.fill(Phy.DigitalBitsPerCycle)(true.B)).asUInt
        }
      }
      io.phy.txTransmitValid.valid := packetsEnqueued(numLanes) << log2Ceil(Phy.DigitalBitsPerCycle) < io.txBitsToSend
      
      for (lane <- 0 to numLanes) {
        val ready = if (lane < numLanes) { io.phy.txTransmitData(lane).ready } else { io.phy.txTransmitValid.ready }
        when (ready) {
          packetsEnqueued(lane) := packetsEnqueued(lane) + 1.U
        }
      }

      when ((0 to numLanes).map(lane => {
          if (lane < numLanes) { !io.phy.txTransmitData(lane).valid } else { !io.phy.txTransmitValid.valid }
        }).reduce(_ && _)) {
        txState := TxTestState.done
      }
    }
    is(TxTestState.done) {
    }
  }

  // RX logic
  
  // Only dequeue if all lanes can be dequeued simultaneously.
  val ready = (0 to numLanes).map(lane => {
          if (lane < numLanes) { !io.phy.rxReceiveData(lane).valid } else { !io.phy.rxReceiveValid.valid }
        }).reduce(_ && _)

  for (lane <- 0 until numLanes) {
    io.phy.rxReceiveData(lane).ready := ready
  }
  io.phy.rxReceiveValid.ready := ready

  // Dumb RX logic (only sets threshold to start recording)
  val recordingStarted = RegInit(false.B)
  val validHighStreak = RegInit(0.U)

  val startRecording = Wire(false.B)
  val startIdx = Wire(0.U)

  // Find correct start index if recording hasn't started already.
  when(!recordingStarted) {
    for (i <- Phy.DigitalBitsPerCycle - 1 to 0 by -1) {
      when ((0 until Phy.DigitalBitsPerCycle).map { j => {
        j.U >= io.rxValidStartThreshold || io.phy.rxReceiveValid.bits(i + j)
      }}.reduce(_&&_)) {
        startRecording := true.B
        startIdx := i.U
      }
    }
  }


  // Insert new values into the register buffers at the appropriate offset.
  recordingStarted := recordingStarted || startRecording
  val newOutputValidBuffer = Wire(Vec(2<<bufferDepthPerLane, Bool()))
  val newOutputDataBuffer = Wire(Vec(numLanes, Vec(2<<bufferDepthPerLane, Bool())))
  newOutputValidBuffer := outputValidBuffer.asTypeOf(newOutputValidBuffer)
  when (recordingStarted || startRecording) {
    for (i <- 0 until Phy.DigitalBitsPerCycle) {
      when (startIdx <= i.U) {
        for (lane <- 0 until numLanes) {
          newOutputDataBuffer(lane)(rxBitsReceived + i.U - startIdx) := io.phy.rxReceiveData(lane).bits(i)
        }
        newOutputValidBuffer(rxBitsReceived + i.U - startIdx) := io.phy.rxReceiveValid.bits(i)
      }
    }
  }
  outputValidBuffer := newOutputValidBuffer.asTypeOf(outputValidBuffer)
  outputDataBuffer := newOutputDataBuffer.asTypeOf(outputDataBuffer)
}

class UciephyTestTL(params: UciephyTestParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("uciephytest", Seq("ucbbar,uciephytest")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  val topIO = BundleBridgeSource(() => new UciephyTopIO(params.numLanes))
  override lazy val module = new UciephyTestImpl
  class UciephyTestImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      // TEST HARNESS
      val test = Module(new UciephyTest(params.bufferDepthPerLane, params.numLanes))

      // MMIO registers.
      val txTestMode = RegInit(0.U(1.W))
      val txValidFramingMode = RegInit(0.U(1.W))
      val txLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(0.U(Phy.SerdesRatio.W))))
      val txManualRepeat = RegInit(0.U(1.W))
      val txFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val txExecute = Wire(DecoupledIO(UInt(1.W)))
      val txBitsToSend = RegInit(0.U(32.W))
      val txDataLane = RegInit(0.U(log2Ceil(params.numLanes).W))
      val txDataOffset = RegInit(0.U((params.bufferDepthPerLane - 6).W))
      val txPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val rxLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(0.U(Phy.SerdesRatio.W))))
      val rxValidStartThreshold = RegInit(4.U(4.W))
      val rxValidStopThreshold = RegInit(4.U(4.W))
      val rxFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val rxDataLane = RegInit(0.U(log2Ceil(params.numLanes).W))
      val rxDataOffset = RegInit(0.U((params.bufferDepthPerLane - 6).W))
      val rxPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val driverPuCtl = RegInit(VecInit(Seq.fill(params.numLanes + 3)(0.U(8.W))))
      val driverPdCtl = RegInit(VecInit(Seq.fill(params.numLanes + 3)(0.U(8.W))))
      val driverEn = RegInit(VecInit(Seq.fill(params.numLanes + 3)(false.B)))
      val phaseCtl = RegInit(VecInit(Seq.fill(params.numLanes + 3)(0.U(8.W))))

      txFsmRst.ready := true.B
      txExecute.ready := true.B
      rxFsmRst.ready := true.B

      test.io.txTestMode := TxTestMode(txTestMode)
      test.io.txValidFramingMode := TxValidFramingMode(txTestMode)
      test.io.txLfsrSeed := txLfsrSeed
      test.io.txManualRepeat := txManualRepeat
      test.io.txFsmRst := txFsmRst.valid
      test.io.txExecute := txExecute.valid
      test.io.txBitsToSend := txBitsToSend
      test.io.txDataLane := txDataLane
      test.io.txDataOffset := txDataOffset
      test.io.txPermute := txPermute
      test.io.rxLfsrSeed := rxLfsrSeed
      test.io.rxValidStartThreshold := rxValidStartThreshold
      test.io.rxValidStopThreshold := rxValidStopThreshold
      test.io.rxFsmRst := rxFsmRst.valid
      test.io.rxDataLane := rxDataLane
      test.io.rxDataOffset := rxDataOffset
      test.io.rxPermute := rxPermute

      // PHY
      val phy = Module(new Phy(params.numLanes))
      phy.io.test <> test.io.phy
      topIO.out(0)._1 <> phy.io.top

      phy.io.driverPuCtl := driverPuCtl
      phy.io.driverPdCtl := driverPuCtl
      phy.io.driverEn := driverEn
      phy.io.phaseCtl := phaseCtl

      val toRegField = (r: UInt) => {
        RegField(r.getWidth, r, RegWriteFn((valid, data) => {
          when (valid) {
            r := data
          }
          true.B
        }), None)
      }

      var mmioRegs = Seq(
        toRegField(txTestMode),
        toRegField(txValidFramingMode),
      ) ++ (0 until params.numLanes).map((i: Int) => {
        toRegField(txLfsrSeed(i))
      }) ++ Seq(
        toRegField(txManualRepeat),
        RegField.w(1, txFsmRst),
        RegField.w(1, txExecute),
        RegField.r(params.bufferDepthPerLane, test.io.txBitsSent),
        toRegField(txBitsToSend),
        toRegField(txDataLane),
        toRegField(txDataOffset),
        RegField.w(64, test.io.txDataChunkIn),
        RegField.r(64, test.io.txDataChunkOut),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(txPermute(i))
      }) ++ Seq(
        RegField.r(2, test.io.txTestState.asUInt),
      ) ++ (0 until params.numLanes).map((i: Int) => {
          toRegField(rxLfsrSeed(i))
      }) ++ Seq(
        toRegField(rxValidStartThreshold),
        toRegField(rxValidStartThreshold),
        RegField.w(1, rxFsmRst),
        RegField.r(params.bufferDepthPerLane, test.io.rxBitsReceived),
        RegField.r(params.bufferDepthPerLane, test.io.rxBitErrors),
        RegField.r(32, test.io.rxSignature),
        toRegField(rxDataLane),
        toRegField(rxDataOffset),
        RegField.r(64, test.io.rxDataChunk),
        RegField.r(64, test.io.rxValidChunk),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(rxPermute(i))
      }) ++ (0 until params.numLanes + 3).flatMap((i: Int) => {
          Seq(
            toRegField(driverPuCtl(i)),
            toRegField(driverPuCtl(i)),
            toRegField(driverEn(i)),
            toRegField(phaseCtl(i))
          )
      })

      node.regmap(mmioRegs.zipWithIndex.map({ case (f, i) => i * 8 -> Seq(f) }): _*)
    }
  }
}


trait CanHavePeripheryUciephyTest { this: BaseSubsystem =>
  private val portName = "uciephytest"

  private val pbus = locateTLBusWrapper(PBUS)

  val uciephy = p(UciephyTestKey) match {
    case Some(params) => {
      val uciephy = LazyModule(new UciephyTestTL(params, pbus.beatBytes)(p))
      uciephy.clockNode := pbus.fixedClockNode
      pbus.coupleTo(portName) { uciephy.node := TLFragmenter(pbus.beatBytes, pbus.blockBytes) := _ }
      Some(uciephy)
    }
    case None => None
  }
}

class WithUciephyTest(params: UciephyTestParams) extends Config((site, here, up) => {
  case UciephyTestKey => Some(params)
})
