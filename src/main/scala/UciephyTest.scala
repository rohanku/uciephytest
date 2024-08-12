package uciephytest

import chisel3._
import chisel3.util._
import chisel3.util.random._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS, SBUS, CacheBlockBytes}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField, RegWriteFn, RegReadFn, RegFieldDesc}
import freechips.rocketchip.tilelink._
import uciephytest.phy.{Phy, PhyToTestIO}
import freechips.rocketchip.util.{AsyncQueueParams}
import testchipip.soc.{OBUS}
import edu.berkeley.cs.ucie.digital.tilelink._
import edu.berkeley.cs.ucie.digital.interfaces.{FdiParams, RdiParams, AfeParams}
import edu.berkeley.cs.ucie.digital.protocol.{ProtocolLayerParams}
import edu.berkeley.cs.ucie.digital.sideband.{SidebandParams}
import edu.berkeley.cs.ucie.digital.logphy.{LinkTrainingParams, TransmitPattern, RegisterRWIO, RegisterRW}

case class UciephyTestParams(
  address: BigInt = 0x4000,
  bufferDepthPerLane: Int = 13,
  numLanes: Int = 2,
  protoParams: ProtocolLayerParams = ProtocolLayerParams(),
  tlParams: TileLinkParams = TileLinkParams(address = 0x0000, addressRange = ((2L << 32)-1), configAddress = 0x7000, inwardQueueDepth = 2, outwardQueueDepth = 2),
  fdiParams: FdiParams = FdiParams(width = 64, dllpWidth = 64, sbWidth = 32),
  rdiParams: RdiParams = RdiParams(width = 64, sbWidth = 32),
  sbParams: SidebandParams = SidebandParams(),
  linkTrainingParams: LinkTrainingParams = LinkTrainingParams(),
  afeParams: AfeParams = AfeParams(sbSerializerRatio = 1,
                                    sbWidth = 1,
                                    mbSerializerRatio = 16,
                                    mbLanes = 16,
                                    STANDALONE = false),
  laneAsyncQueueParams: AsyncQueueParams = AsyncQueueParams()
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

class UciephyTopIO(numLanes: Int = 2) extends Bundle {
  val txData = Output(Vec(numLanes, Bool()))
  val txValid = Output(Bool())
  val refClkP = Input(Clock())
  val refClkN = Input(Clock())
  val txClkP = Output(Clock())
  val txClkN = Output(Clock())
  val rxData = Input(Vec(numLanes, Bool()))
  val rxValid = Input(Bool())
  val rxClkP = Input(Clock())
  val rxClkN = Input(Clock())
  val sbTxClk = Output(Clock())
  val sbTxData = Output(Bool())
  val sbRxClk = Input(Clock())
  val sbRxData = Input(Bool())
  val pllIref = Input(Bool())
}



class UciephyTestMMIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Bundle {
  // TX CONTROL
  // =====================
  // The test mode of the TX.
  val txTestMode = Input(TxTestMode())
  // The valid framing mode of the TX.
  val txValidFramingMode = Input(TxValidFramingMode())
  // Seed of the TX LFSR.
  val txLfsrSeed = Input(Vec(numLanes, UInt((2 * Phy.DigitalBitsPerCycle).W)))
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
  // Data chunk lane group in input buffer. Each lane group consists of 4 adjacent lanes (e.g. 0, 1, 2, 3).
  val txDataLaneGroup = Input(UInt((log2Ceil(numLanes) - 2).max(1).W))
  // Data chunk offset in input buffer.
  val txDataOffset = Input(UInt((bufferDepthPerLane - 5).W))
  // 128-bit data chunk to write (32 bits per lane).
  val txDataChunkIn = Flipped(DecoupledIO(UInt(128.W)))
  // Data chunk at the given chunk offset for inspecting the data to be sent. Only available in idle/done mode.
  val txDataChunkOut = Output(UInt(128.W))
  // Permutation of data fed into the serializer, in case serializer timing is off.
  val txPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))
  // State of the TX test FSM.
  val txTestState = Output(TxTestState())

  // RX CONTROL
  // ====================
  // Seed of the RX LFSR used for detecting bit errors. Should be the same as the TX seed of the transmitting chiplet.
  val rxLfsrSeed = Input(Vec(numLanes, UInt((2 * Phy.DigitalBitsPerCycle).W)))
  // The number of bit periods that valid must be high for the transmission to be considered valid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 1 such that data is received as long
  // as valid goes high for at least 1 bit period.
  val rxValidStartThreshold = Input(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  // The number of bit periods that valid must be low for the transmission to be considered invalid. By default,
  // this is 4 since UCIe specifies asserting valid for 4 bit periods then de-asserting valid for 4 bit periods
  // during transmission. If the valid transmission is buggy, can set this to 0 so that
  // data starts being received as soon as valid goes high and ignores any glitches afterwards.
  val rxValidStopThreshold = Input(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  // Resets the RX FSM (i.e. resetting the number of bits received and the offset within the output
  // buffer to 0).
  val rxFsmRst = Input(Bool())
  // The number of bits received since the last FSM reset. Only the first 2^bufferDepthPerLane bits received 
  // per lane are stored in the output buffer.
  val rxBitsReceived = Output(UInt((bufferDepthPerLane + 1).W))
  // The number of bit errors per lane since the last FSM reset. Only applicable in `TxTestMode.lsfr`.
  val rxBitErrors = Output(Vec(numLanes, UInt((bufferDepthPerLane + 1).W)))
  // Pause the `rxBitsReceived` and `rxBitErrors` counters to read them atomically.
  val rxPauseCounters = Input(Bool())
  // A MISR derived from the bits received since the last FSM reset.
  val rxSignature = Output(UInt(32.W))
  // Data chunk lane in output buffer.
  val rxDataLane = Input(UInt(log2Ceil(numLanes).W))
  // Data chunk offset in output buffer.
  val rxDataOffset = Input(UInt((bufferDepthPerLane - 5).W))
  // Data chunk at the given chunk offset for inspect the received data.
  val rxDataChunk = Output(UInt(32.W))
  // Valid chunk at the given chunk offset for inspect the valid signals corresponding to the received data.
  // In a correct UCIe implementation, there should be 4 1-bits followed by 4 0-bits repeated across the entire 
  // transmission.
  val rxValidChunk = Output(UInt(32.W))
  // Permutation of data read from the deserializer, in case deserializer timing is off.
  val rxPermute = Input(Vec(Phy.SerdesRatio, UInt(log2Ceil(Phy.SerdesRatio).W)))
}

class UciephyTestIO(bufferDepthPerLane: Int = 10, numLanes: Int = 2) extends Bundle {
  val mmio = new UciephyTestMMIO(bufferDepthPerLane, numLanes)

  // PHY INTERFACE
  // ====================
  val phy = Flipped(new PhyToTestIO(numLanes))
}

class UciephyTest(bufferDepthPerLane: Int = 10, numLanes: Int = 2, sim: Boolean = false) extends Module {
  val io = IO(new UciephyTestIO(bufferDepthPerLane, numLanes))

  // TX registers
  val txReset = io.mmio.txFsmRst || reset.asBool
  val txState = withReset(txReset) { RegInit(TxTestState.idle) }
  val packetsEnqueued = withReset(txReset) { RegInit(0.U(bufferDepthPerLane.W)) }
  val txLfsrs = (0 until numLanes).map((i: Int) => {
    val lfsr = Module(
      new FibonacciLFSR(
        2 * Phy.DigitalBitsPerCycle,
        taps = LFSR.tapsMaxPeriod.get(2 * Phy.DigitalBitsPerCycle).get.head,
        step = Phy.DigitalBitsPerCycle,
      ),
    )
    lfsr.io.seed.bits := io.mmio.txLfsrSeed(i).asTypeOf(lfsr.io.seed.bits)
    lfsr.io.seed.valid := txReset
    lfsr.io.increment := false.B
    lfsr
  })
  val loadedFirstChunk = withReset(txReset) { RegInit(false.B) }
  val nextChunk = withReset(txReset) { RegInit(0.U(32.W)) }

  // RX registers
  val rxReset = io.mmio.rxFsmRst || reset.asBool
  val rxBitsReceived = withReset(rxReset) { RegInit(0.U((bufferDepthPerLane + 1).W)) }
  val rxBitErrors = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U((bufferDepthPerLane + 1).W)))) }
  val rxBitsReceivedOutput = withReset(rxReset) { RegInit(0.U((bufferDepthPerLane + 1).W)) }
  val rxErrorMask = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U(Phy.DigitalBitsPerCycle.W)))) }
  val rxBitErrorsOutput = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U((bufferDepthPerLane + 1).W)))) }
  val rxPrevLfsrs = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U((bufferDepthPerLane + 1).W)))) }
  val rxLfsrs = (0 until numLanes).map((i: Int) => {
    val lfsr = Module(
      new FibonacciLFSR(
        2 * Phy.DigitalBitsPerCycle,
        taps = LFSR.tapsMaxPeriod.get(2 * Phy.DigitalBitsPerCycle).get.head,
        step = Phy.DigitalBitsPerCycle,
      ),
    )
    lfsr.io.seed.bits := io.mmio.rxLfsrSeed(i).asTypeOf(lfsr.io.seed.bits)
    lfsr.io.seed.valid := txReset
    lfsr.io.increment := false.B
    lfsr
  })
  
  val rxSignature = withReset(rxReset) { RegInit(0.U(32.W)) }

  val numInputSrams = (numLanes - 1)/4 + 1
  val numOutputSrams = numLanes/4 + 1
  val inputBuffer = (0 until numInputSrams).map(i => SyncReadMem(1 << (bufferDepthPerLane - 5), UInt(128.W)))
  val inputBufferAddr = Wire(UInt(log2Ceil(1 << (bufferDepthPerLane - 6)).W))
  inputBufferAddr := io.mmio.txDataOffset
  val inputRdPorts = (0 until numInputSrams).map(i => inputBuffer(i)(inputBufferAddr))
  val inputWrPorts = (0 until numInputSrams).map(i => inputBuffer(i)(io.mmio.txDataOffset))
  val outputBuffer = (0 until numOutputSrams).map(i => SyncReadMem(1 << (bufferDepthPerLane - 5),  UInt(128.W)))
  val outputBufferAddr = Wire(UInt(log2Ceil(1 << (bufferDepthPerLane - 5)).W))
  outputBufferAddr := io.mmio.rxDataOffset
  val outputRdPorts = (0 until numOutputSrams).map(i => outputBuffer(i)(io.mmio.rxDataOffset))
  val outputWrPorts = (0 until numOutputSrams).map(i => outputBuffer(i)(outputBufferAddr))

  io.mmio.txBitsSent := packetsEnqueued << log2Ceil(Phy.DigitalBitsPerCycle)
  io.mmio.txDataChunkIn.ready := txState === TxTestState.idle
  io.mmio.txDataChunkOut := 0.U
  for (i <- 0 until numInputSrams) {
    when (i.U === io.mmio.txDataLaneGroup) {
      io.mmio.txDataChunkOut := inputRdPorts(i)
    }
  }
  io.mmio.txTestState := txState

  when (io.mmio.rxPauseCounters) {
    rxBitsReceivedOutput := rxBitsReceivedOutput
    for (i <- 0 until numLanes) {
      rxBitErrorsOutput(i) := rxBitErrorsOutput(i)
    }
  } .otherwise {
    rxBitsReceivedOutput := rxBitsReceived
    for (i <- 0 until numLanes) {
      rxBitErrorsOutput(i) := rxBitErrors(i) + PopCount(rxErrorMask(i))
    }
  }
  io.mmio.rxBitsReceived := rxBitsReceivedOutput
  io.mmio.rxBitErrors := rxBitErrorsOutput
  io.mmio.rxSignature := rxSignature
  io.mmio.rxDataChunk := 0.U
  for (i <- 0 until numOutputSrams) {
    when (i.U === io.mmio.rxDataLane >> 2.U) {
      io.mmio.rxDataChunk := outputRdPorts(i).asTypeOf(Vec(4, UInt(32.W)))(io.mmio.rxDataLane % 4.U)
    }
  }
  io.mmio.rxValidChunk := outputRdPorts(numLanes >> 2).asTypeOf(Vec(4, UInt(32.W)))(numLanes % 4)

  for (lane <- 0 until numLanes) {
    io.phy.tx.bits.data(lane) := 0.U
  }
  io.phy.tx.bits.valid := 0.U
  io.phy.rx.ready := false.B
  io.phy.tx.valid := false.B
  io.phy.txRst := txReset
  io.phy.rxRst := rxReset

  // TX logic
  switch(txState) {
    is(TxTestState.idle) {
      when (io.mmio.txDataChunkIn.valid) {
        for (i <- 0 until numInputSrams) {
          when (i.U === io.mmio.txDataLaneGroup) {
              inputWrPorts(i) := io.mmio.txDataChunkIn.bits
          }
        }
      }

      when (io.mmio.txExecute) {
        txState := TxTestState.run
      }
    }
    is(TxTestState.run) {
      switch (io.mmio.txTestMode) {
        is (TxTestMode.manual) {
          when (loadedFirstChunk) {
            when (io.phy.tx.ready) {
              inputBufferAddr := packetsEnqueued + 1.U
            } .otherwise {
              inputBufferAddr := packetsEnqueued
            }
            for (lane <- 0 until numLanes) {
              io.phy.tx.bits.data(lane) := inputRdPorts(lane >> 2).asTypeOf(Vec(4, UInt(32.W)))(lane % 4)
            }
            io.phy.tx.valid := (packetsEnqueued << log2Ceil(Phy.DigitalBitsPerCycle)) < io.mmio.txBitsToSend
          } .otherwise {
            inputBufferAddr := 0.U
            loadedFirstChunk := true.B
          }
        }
        is (TxTestMode.lfsr) {
          for (lane <- 0 until numLanes) {
            io.phy.tx.bits.data(lane) := Reverse(txLfsrs(lane).io.out.asUInt)(31, 0).asTypeOf(io.phy.tx.bits.data(lane))
          }
          io.phy.tx.valid := true.B
        }
      }
      when (io.phy.tx.valid) {
        switch(io.mmio.txValidFramingMode) {
          is (TxValidFramingMode.ucie) {
            io.phy.tx.bits.valid := VecInit((0 until Phy.DigitalBitsPerCycle/8).flatMap(_ => Seq.fill(4)(true.B) ++ Seq.fill(4)(false.B))).asUInt
          }
          is (TxValidFramingMode.simple) {
            io.phy.tx.bits.valid := VecInit(Seq.fill(Phy.DigitalBitsPerCycle)(true.B)).asUInt
          }
        }
      }
      
      when (io.phy.tx.valid && io.phy.tx.ready) {
        switch (io.mmio.txTestMode) {
          is (TxTestMode.manual) {
            packetsEnqueued := packetsEnqueued + 1.U
          }
          is (TxTestMode.lfsr) {
            for (lane <- 0 until numLanes) {
              txLfsrs(lane).io.increment := true.B
            }
          }
        }
      }

      when (loadedFirstChunk && !io.phy.tx.valid) {
        txState := TxTestState.done
      }
    }
    is(TxTestState.done) {
    }
  }

  // RX logic
  
  io.phy.rx.ready := true.B

  // Dumb RX logic (only sets threshold to start recording)
  val recordingStarted = withReset(rxReset) { RegInit(false.B) }
  val validHighStreak = withReset(rxReset) { RegInit(0.U(log2Ceil(Phy.DigitalBitsPerCycle).W)) }
  val prevDataBits = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U(Phy.DigitalBitsPerCycle.W)))) }

  val startRecording = Wire(Bool())
  val startIdx = Wire(UInt(log2Ceil(Phy.DigitalBitsPerCycle).W))
  startRecording := false.B
  startIdx := 0.U

  val runningData = withReset(rxReset) { RegInit(VecInit(Seq.fill(numLanes)(0.U(64.W)))) }
  val runningValid = withReset(rxReset) { RegInit(0.U(64.W)) } 
  runningData := runningData
  runningValid := runningValid

  // Fix timing issues by pipelining computation
  val rxValidStartThresholdMask = RegInit(0.U(Phy.DigitalBitsPerCycle.W))
  rxValidStartThresholdMask := (1.U << io.mmio.rxValidStartThreshold) - 1.U

  for (i <- 0 until numLanes) {
    rxErrorMask(i) := 0.U
  }

  // Check valid streak after each packet is dequeued.
  when (io.phy.rx.ready & io.phy.rx.valid) {
    // Store data in a register
    for (lane <- 0 until numLanes) {
      prevDataBits(lane) := io.phy.rx.bits.data(lane)
    }

    // Find correct start index if recording hasn't started already.
    for (i <- Phy.DigitalBitsPerCycle - 1 to 0 by -1) {
      val shouldStartRecording = ((io.phy.rx.bits.valid >> i.U) & rxValidStartThresholdMask) === rxValidStartThresholdMask
      when(!recordingStarted && shouldStartRecording) {
        startRecording := true.B
        startIdx := i.U
      }
    }

    for (i <- Phy.DigitalBitsPerCycle to 0 by -1) {
      val mask = Wire(UInt(Phy.DigitalBitsPerCycle.W))
      mask := ~((1.U << i.U) - 1.U)
      when ((io.phy.rx.bits.valid & mask) === mask) {
        validHighStreak := (Phy.DigitalBitsPerCycle - i).U
      }
    }

    recordingStarted := recordingStarted || startRecording


    val rxBitsReceivedOffset = rxBitsReceived % 32.U
    val rxBlock = rxBitsReceived >> 5.U
    // Insert new values into the register buffers at the appropriate offset.
    when (!recordingStarted && !startRecording) {
      // Store latest data at the beginning of the `runningData` register.
      for (lane <- 0 until numLanes) {
        runningData(lane) := io.phy.rx.bits.data(lane)
      }
      runningValid := io.phy.rx.bits.valid
    } .elsewhen (startIdx === 0.U && startRecording) {
      // If we are starting recording where the whole received data is valid, append to the second
      // half of `runningData` and shift out the bad bits. Since we now have at least 32 bits of valid data,
      // we write the first 32 bits to the output SRAM.
      val numExtraBits = Phy.DigitalBitsPerCycle.U - validHighStreak
      val dataOffset = validHighStreak
      val prevMask = Wire(UInt(64.W))
      prevMask := ((1.U << validHighStreak) - 1.U)
      val dataMask = Wire(UInt(64.W))
      dataMask := ((1.U << Phy.DigitalBitsPerCycle.U) - 1.U) << dataOffset

      outputBufferAddr := 0.U
      val toWrite = (0 until numOutputSrams).map(i => {
        val wire = Wire(Vec(4, UInt(32.W)))
        for (i <- 0 until 4) {
          wire(i) := 0.U
        }
        wire
      })
      for (lane <- 0 until numLanes) {
        val prev = Wire(UInt(64.W))
        prev := runningData(lane) >> numExtraBits
        val data = Wire(UInt(64.W))
        data := io.phy.rx.bits.data(lane) << dataOffset
        val newData = Wire(UInt(64.W))
        newData := (prev & prevMask) | (data & dataMask)

        // Compare data against LFSR and increment LFSR.
        rxLfsrs(lane).io.increment := true.B
        val maskedLfsr = Reverse(rxLfsrs(lane).io.out.asUInt) & (prevMask | dataMask)
        val maskedData = (prev & prevMask) | (data & dataMask)
        rxErrorMask(lane) := maskedLfsr ^ maskedData
        rxBitErrors(lane) := rxBitErrors(lane) + PopCount(rxErrorMask(lane))

        toWrite(lane >> 2)(lane % 4) := newData(31, 0)
        runningData(lane) := newData >> 32.U
      }
      val data = Wire(UInt(64.W))
      data := io.phy.rx.bits.valid << dataOffset
      val newData = Wire(UInt(64.W))
      newData := prevMask | (data & dataMask)
      toWrite(numLanes >> 2)(numLanes % 4) := newData(31, 0)
      for (i <- 0 until numOutputSrams) {
        outputWrPorts(i) := toWrite(i).asTypeOf(outputWrPorts(i))
      }
      runningValid := newData >> 32.U
      rxBitsReceived := rxBitsReceived +& Phy.DigitalBitsPerCycle.U +& validHighStreak
    } .otherwise {
      when (recordingStarted || startRecording) {
        val shouldWrite = rxBlock < (1 << (bufferDepthPerLane - 5)).U && rxBitsReceivedOffset +& Phy.DigitalBitsPerCycle.U - startIdx >= 32.U
        when(shouldWrite) {
          outputBufferAddr := rxBlock
        }
        val dataMask = Wire(UInt(64.W))
        dataMask := ((1.U << (Phy.DigitalBitsPerCycle.U - startIdx)) - 1.U) << rxBitsReceivedOffset
        val keepMask = Wire(UInt(64.W))
        keepMask := ~dataMask
        val toWrite = (0 until numOutputSrams).map(i => {
          val wire = Wire(Vec(4, UInt(32.W)))
          for (i <- 0 until 4) {
            wire(i) := 0.U
          }
          wire
        })
        for (lane <- 0 until numLanes) {
          val data = Wire(UInt(64.W))
          data := (io.phy.rx.bits.data(lane) << rxBitsReceivedOffset) >> startIdx
          val newData = Wire(UInt(64.W))
          newData := (data & dataMask) | (runningData(lane) & keepMask)
          runningData(lane) := newData
          when(shouldWrite) {
            toWrite(lane >> 2)(lane % 4) := newData(31, 0)
            runningData(lane) := newData >> 32.U
          }

          // Compare data against LFSR and increment LFSR.
          val lfsrOffset = rxBitsReceivedOffset % Phy.DigitalBitsPerCycle.U
          rxLfsrs(lane).io.increment := (Phy.DigitalBitsPerCycle.U - startIdx +& rxBitsReceivedOffset) >= 32.U
          val lfsrData = ((data & dataMask) >> rxBitsReceivedOffset) << lfsrOffset
          val lfsrMask = (dataMask >> rxBitsReceivedOffset) << lfsrOffset
          val maskedLfsr = Reverse(rxLfsrs(lane).io.out.asUInt) & lfsrMask
          rxErrorMask(lane) := lfsrData ^ maskedLfsr
          rxBitErrors(lane) := rxBitErrors(lane) + PopCount(rxErrorMask(lane))
        }
        val data = Wire(UInt(64.W))
        data := (io.phy.rx.bits.valid << rxBitsReceivedOffset) >> startIdx
        val newData = Wire(UInt(64.W))
        newData := (data & dataMask) | (runningValid & keepMask)
        runningValid := newData
        when(shouldWrite) {
          toWrite(numLanes >> 2)(numLanes % 4) := newData(31, 0)
          runningValid := newData >> 32.U
          for (i <- 0 until numOutputSrams) {
            outputWrPorts(i) := toWrite(i).asTypeOf(outputWrPorts(i))
          }
        }

        rxBitsReceived := rxBitsReceived +& Phy.DigitalBitsPerCycle.U - startIdx
      }
    }
  }
}

class UciephyTestTL(params: UciephyTestParams, beatBytes: Int)(implicit p: Parameters) extends ClockSinkDomain(ClockSinkParameters())(p) {
  val device = new SimpleDevice("uciephytest", Seq("ucbbar,uciephytest")) 
  val node = TLRegisterNode(Seq(AddressSet(params.address, 4096-1)), device, "reg/control", beatBytes=beatBytes)

  val topIO = BundleBridgeSource(() => new UciephyTopIO(params.numLanes))

  val uciTL = LazyModule(new UCITLFront(
        tlParams    = params.tlParams,
        protoParams = params.protoParams,
        fdiParams   = params.fdiParams,
        rdiParams   = params.rdiParams,
        sbParams    = params.sbParams,
        //myId        = myId,
        linkTrainingParams = params.linkTrainingParams,
        afeParams   = params.afeParams,
        laneAsyncQueueParams = params.laneAsyncQueueParams
      ))

  override lazy val module = new UciephyTestImpl
  class UciephyTestImpl extends Impl {
    val io = IO(new Bundle {})
    withClockAndReset(clock, reset) {
      // TEST HARNESS
      val test = Module(new UciephyTest(params.bufferDepthPerLane, params.numLanes))

      // MMIO registers.
      val txTestMode = RegInit(0.U(1.W))
      val txValidFramingMode = RegInit(0.U(1.W))
      val txLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(1.U(Phy.SerdesRatio.W))))
      val txManualRepeat = RegInit(0.U(1.W))
      val txFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val txExecute = Wire(DecoupledIO(UInt(1.W)))
      val txBitsToSend = RegInit(0.U(32.W))
      val txDataLaneGroup = RegInit(0.U((log2Ceil(params.numLanes) - 2).max(1).W))
      val txDataOffset = RegInit(0.U((params.bufferDepthPerLane - 6).W))
      val txPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val rxLfsrSeed = RegInit(VecInit(Seq.fill(params.numLanes)(1.U(Phy.SerdesRatio.W))))
      val rxValidStartThreshold = RegInit(4.U(log2Ceil(Phy.DigitalBitsPerCycle).W))
      val rxValidStopThreshold = RegInit(4.U(log2Ceil(Phy.DigitalBitsPerCycle).W))
      val rxFsmRst = Wire(DecoupledIO(UInt(1.W)))
      val rxPauseCounters = RegInit(0.U(1.W))
      val rxDataLane = RegInit(0.U(log2Ceil(params.numLanes).W))
      val rxDataOffset = RegInit(0.U((params.bufferDepthPerLane - 5).W))
      val rxPermute = RegInit(VecInit((0 until Phy.SerdesRatio).map(_.U(log2Ceil(Phy.SerdesRatio).W))))
      val driverPuCtl = RegInit(VecInit(Seq.fill(params.numLanes + 5)(0.U(6.W))))
      val driverPdCtl = RegInit(VecInit(Seq.fill(params.numLanes + 5)(0.U(6.W))))
      val driverEn = RegInit(VecInit(Seq.fill(params.numLanes + 5)(false.B)))
      val clockingPiCtl = RegInit(VecInit(Seq.fill(params.numLanes + 1)(0.U(52.W))))
      val clockingMiscCtl = RegInit(VecInit(Seq.fill(params.numLanes + 1)(0.U(28.W))))
      val shufflerCtl = RegInit(VecInit(Seq.fill(params.numLanes + 1)(
        VecInit((0 until 16).map(i => i.U(4.W))).asUInt
      )))
      val terminationCtl = RegInit(VecInit(Seq.fill(params.numLanes + 3)(~0.U(6.W))))
      val vrefCtl = RegInit(VecInit(Seq.fill(params.numLanes + 1)(0.U(64.W))))
      val dllCode = Wire(Vec(params.numLanes + 1, UInt(10.W)))
      // UCIe logphy related
      val ucieStack = RegInit(false.B)
      val maxPatternCountWidth = log2Ceil(params.linkTrainingParams.maxPatternCount + 1)
      val pattern = RegInit(0.U(2.W))
      val patternUICount = RegInit(0.U(maxPatternCountWidth.W))
      val triggerNew = new RegisterRW(false.B, "triggerNew") 
      val triggerExit = new RegisterRW(false.B, "triggerExit") 
      val outputValid = RegInit(false.B)
      val errorCounts = RegInit(VecInit(Seq.fill(params.afeParams.mbLanes)(0.U(maxPatternCountWidth.W))))

      txFsmRst.ready := true.B
      txExecute.ready := true.B
      rxFsmRst.ready := true.B

      test.io.mmio.txTestMode := TxTestMode(txTestMode)
      test.io.mmio.txValidFramingMode := TxValidFramingMode(txTestMode)
      test.io.mmio.txLfsrSeed := txLfsrSeed
      test.io.mmio.txManualRepeat := txManualRepeat
      test.io.mmio.txFsmRst := txFsmRst.valid
      test.io.mmio.txExecute := txExecute.valid
      test.io.mmio.txBitsToSend := txBitsToSend
      test.io.mmio.txDataLaneGroup := txDataLaneGroup
      test.io.mmio.txDataOffset := txDataOffset
      test.io.mmio.txPermute := txPermute
      test.io.mmio.rxLfsrSeed := rxLfsrSeed
      test.io.mmio.rxValidStartThreshold := rxValidStartThreshold
      test.io.mmio.rxValidStopThreshold := rxValidStopThreshold
      test.io.mmio.rxFsmRst := rxFsmRst.valid
      test.io.mmio.rxPauseCounters := rxPauseCounters
      test.io.mmio.rxDataLane := rxDataLane
      test.io.mmio.rxDataOffset := rxDataOffset
      test.io.mmio.rxPermute := rxPermute

      // PHY
      val phy = Module(new Phy(params.numLanes))
      when (ucieStack) { 

        phy.io.test.tx <> uciTL.module.io.phyAfe.get.tx.map(f => {
          val x = Wire(chiselTypeOf(phy.io.test.tx.bits))
          x.data := f.data
          x.valid := f.valid.asUInt
          x
        })

        phy.io.test.rx.map(f => {
          val x = Wire(chiselTypeOf(uciTL.module.io.phyAfe.get.rx.bits))
          x.data := f.data
          x.valid := f.valid.asTypeOf(x.valid)
          x
        }) <> uciTL.module.io.phyAfe.get.rx
        
        phy.io.test.txRst <> uciTL.module.io.phyAfe.get.txRst
        phy.io.test.rxRst <> uciTL.module.io.phyAfe.get.rxRst

        phy.io.sideband.txClk := uciTL.module.io.txSbAfe.clk
        phy.io.sideband.txData := uciTL.module.io.txSbAfe.data
        uciTL.module.io.rxSbAfe.clk := phy.io.sideband.rxClk
        uciTL.module.io.rxSbAfe.data := phy.io.sideband.rxData

        test.io.phy.tx.nodeq()
        test.io.phy.rx.noenq()
      }.otherwise {
        phy.io.test <> test.io.phy
        uciTL.module.io.phyAfe.get.tx.nodeq()
        uciTL.module.io.phyAfe.get.rx.noenq()
        uciTL.module.io.rxSbAfe.clk := 0.U
        uciTL.module.io.rxSbAfe.data := 0.U.asTypeOf(uciTL.module.io.rxSbAfe.data)
      }

      uciTL.module.io.train.get.pattern := pattern.asTypeOf(TransmitPattern())
      uciTL.module.io.train.get.patternUICount := patternUICount
      triggerNew.connect(uciTL.module.io.train.get.triggerNew)
      triggerExit.connect(uciTL.module.io.train.get.triggerExit)
      outputValid := uciTL.module.io.train.get.outputValid
      errorCounts := uciTL.module.io.train.get.errorCounts

      topIO.out(0)._1 <> phy.io.top
      // Tie sideband to 0 for simple test
      phy.io.sideband.txClk := false.B
      phy.io.sideband.txData := false.B

      phy.io.driverPuCtl := driverPuCtl
      phy.io.driverPdCtl := driverPdCtl
      phy.io.driverEn := driverEn
      phy.io.clockingPiCtl := clockingPiCtl
      phy.io.clockingMiscCtl := clockingMiscCtl
      phy.io.terminationCtl := terminationCtl
      phy.io.shufflerCtl := shufflerCtl.asTypeOf(phy.io.shufflerCtl)
      phy.io.vrefCtl := vrefCtl
      for (lane <- 0 to params.numLanes) {
        dllCode(lane) := Cat(phy.io.dllCode(lane), phy.io.dllCodeb(lane))
      }

      val toRegField = (r: UInt) => {
        RegField(r.getWidth, r, RegWriteFn((valid, data) => {
          when (valid) {
            r := data
          }
          true.B
        }), None)
      }

      val regIO = IO(new Bundle {
        val triggerNew = triggerNew.io
        val triggerExit = triggerExit.io
      })

      var mmioRegs = Seq(
        toRegField(txTestMode),
        toRegField(txValidFramingMode),
      ) ++ (0 until params.numLanes).map((i: Int) => {
        toRegField(txLfsrSeed(i))
      }) ++ Seq(
        toRegField(txManualRepeat),
        RegField.w(1, txFsmRst),
        RegField.w(1, txExecute),
        RegField.r(params.bufferDepthPerLane, test.io.mmio.txBitsSent),
        toRegField(txBitsToSend),
        toRegField(txDataLaneGroup),
        toRegField(txDataOffset),
        RegField.w(64, test.io.mmio.txDataChunkIn),
        RegField.r(64, test.io.mmio.txDataChunkOut),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(txPermute(i))
      }) ++ Seq(
        RegField.r(2, test.io.mmio.txTestState.asUInt),
      ) ++ (0 until params.numLanes).map((i: Int) => {
          toRegField(rxLfsrSeed(i))
      }) ++ (0 until params.numLanes).map((i: Int) => {
        RegField.r(params.bufferDepthPerLane + 1, test.io.mmio.rxBitErrors(i))
      }) ++ Seq(
        toRegField(rxValidStartThreshold),
        toRegField(rxValidStartThreshold),
        RegField.w(1, rxFsmRst),
        toRegField(rxPauseCounters),
        RegField.r(params.bufferDepthPerLane, test.io.mmio.rxBitsReceived),
        RegField.r(32, test.io.mmio.rxSignature),
        toRegField(rxDataLane),
        toRegField(rxDataOffset),
        RegField.r(32, test.io.mmio.rxDataChunk),
        RegField.r(32, test.io.mmio.rxValidChunk),
      ) ++ (0 until Phy.SerdesRatio).map((i: Int) => {
          toRegField(rxPermute(i))
      }) ++ (0 until params.numLanes + 5).flatMap((i: Int) => {
          Seq(
            toRegField(driverPuCtl(i)),
            toRegField(driverPuCtl(i)),
            toRegField(driverEn(i)),
          )
      }) ++ (0 until params.numLanes + 1).flatMap((i: Int) => {
          Seq(
            toRegField(clockingPiCtl(i)),
            toRegField(clockingMiscCtl(i)),
            RegField.r(10, dllCode(i))
          )
      }) ++ (0 until params.numLanes + 1).flatMap((i: Int) => {
          Seq(
            toRegField(shufflerCtl(i)),
          )
      }) ++ (0 until params.numLanes + 3).flatMap((i: Int) => {
          Seq(
            toRegField(terminationCtl(i)),
          )
      }) ++ (0 until params.numLanes + 1).flatMap((i: Int) => {
          Seq(
            toRegField(vrefCtl(i)),
          )
      }) ++ Seq(
        RegField.w(1, ucieStack),
        RegField.r(1, outputValid)
      ) ++ (0 until params.numLanes).map((i: Int) => {
        RegField.r(maxPatternCountWidth, errorCounts(i))
      }) ++ Seq(
        RegField.w(2, pattern),
        RegField.w(32, patternUICount),
        regIO.triggerNew.regField(RegFieldDesc("triggerNew", "training triggered")),
        regIO.triggerExit.regField(RegFieldDesc("triggerExit", "training exited"))
      )

      node.regmap(mmioRegs.zipWithIndex.map({ case (f, i) => i * 8 -> Seq(f) }): _*)
    }
  }
}


trait CanHavePeripheryUciephyTest { this: BaseSubsystem =>
  private val portName = "uciephytest"

  private val pbus = locateTLBusWrapper(PBUS)
  private val obus = locateTLBusWrapper(OBUS) //TODO: make parameterizable?
  private val sbus = locateTLBusWrapper(SBUS)

  val uciephy = p(UciephyTestKey) match {
    case Some(params) => {
      val uciephy = LazyModule(new UciephyTestTL(params, sbus.beatBytes)(p))
      uciephy.clockNode := sbus.fixedClockNode
      sbus.coupleTo(portName) { uciephy.node := TLBuffer() := TLFragmenter(sbus.beatBytes, sbus.blockBytes) := TLBuffer() := _ }
      uciephy.uciTL.clockNode := sbus.fixedClockNode
      obus.coupleTo(s"ucie_tl_man_port") { 
          uciephy.uciTL.managerNode := TLWidthWidget(obus.beatBytes) := TLBuffer() := TLSourceShrinker(params.tlParams.sourceIDWidth) := TLFragmenter(obus.beatBytes, p(CacheBlockBytes)) := TLBuffer() := _ 
      } //manager node because SBUS is making request?
      sbus.coupleFrom(s"ucie_tl_cl_port") { _ := TLBuffer() := TLWidthWidget(sbus.beatBytes) := TLBuffer() := uciephy.uciTL.clientNode }
      sbus.coupleTo(s"ucie_tl_ctrl_port") { uciephy.uciTL.regNode.node := TLWidthWidget(sbus.beatBytes) := TLFragmenter(sbus.beatBytes, sbus.blockBytes) := TLBuffer() := _ }

      Some(uciephy)
    }
    case None => None
  }
}

class WithUciephyTest(params: UciephyTestParams) extends Config((site, here, up) => {
  case UciephyTestKey => Some(params)
})
