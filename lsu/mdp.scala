package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.Str

import boom.common._

case class LWTConfig(
  numEntries: Int = 1024,
  predictionWidth: Int = 1,
  pcStartBit: Int = 1,
  clearCyclePeriod: Int = 2097152  // 自动清零的周期，单位为时钟周期，默认2^25(33554432≈40M)个周期清一次
) {
  require(isPow2(numEntries), "表大小必须是2的幂")
  require(clearCyclePeriod > 0, "清零周期必须大于0")
  
  val indexBits: Int = log2Ceil(numEntries)
  val pcEndBit: Int = pcStartBit + indexBits - 1
  val clearCycleBits: Int = log2Ceil(clearCyclePeriod)  // 计时器所需的位数
  
  def extractIndex(pc: UInt): UInt = {
    require(pc.getWidth > pcEndBit, s"PC宽度${pc.getWidth}不够，需要至少${pcEndBit+1}位")
    pc(pcEndBit, pcStartBit)
  }
}

class LWT_Entry(val predictWidth: Int = 1)(implicit p: Parameters) extends BoomBundle()(p)
{
    val predict = UInt(predictWidth.W)
}