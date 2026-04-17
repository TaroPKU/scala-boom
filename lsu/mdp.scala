package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import boom.common._

object MDP_GHistory_length {
  val length = 4
}

class MDP(implicit p: Parameters) {

  val numL1Entries = 128 // 比如PC直接映射
  val numL2Entries = 128  // PC ^ ghist 映射
  val numL3Entries = 128

  // L1 表项，3bit记录存储距离 (000: 无效, +1 偏移)
  val l1_table = RegInit(VecInit(Seq.fill(numL1Entries)(0.U(3.W))))
  // L2 表项，3bit记录存储距离
  val l2_table = RegInit(VecInit(Seq.fill(numL2Entries)(0.U(3.W))))
  // 最顶层 L3 表项 (等待全部前序指令)，1bit
  val l3_table = RegInit(VecInit(Seq.fill(numL3Entries)(0.U(1.W))))

  // 独立清零计数器
  val clearPeriod = 1048576
  val clearCtr = RegInit(0.U(log2Ceil(clearPeriod).W))
  val shouldClear = clearCtr === (clearPeriod - 1).U
  clearCtr := Mux(shouldClear, 0.U, clearCtr + 1.U)

  when(shouldClear) {
    l1_table.foreach(_ := 0.U)
    l2_table.foreach(_ := 0.U)
    l3_table.foreach(_ := 0.U)
  }

  def get_l1_idx(pc: UInt) = pc(log2Ceil(numL1Entries), 1)
  def get_l2_idx(pc: UInt, ghist: UInt, xorBits: Int = 2) = {
    val idxBits = log2Ceil(numL2Entries)
    val lowBits = idxBits - xorBits
    
    val pc_low  = pc(lowBits, 1)
    val pc_high = pc(idxBits, lowBits + 1)
    Cat(pc_high ^ ghist(xorBits - 1, 0), pc_low)
  }
  def get_l3_idx(pc: UInt, ghist: UInt) = {
    val idxBits = log2Ceil(numL3Entries)
    val xorBits = MDP_GHistory_length.length
    val lowBits = idxBits - xorBits
    
    val pc_low  = pc(lowBits, 1)
    val pc_high = pc(idxBits, lowBits + 1)
    Cat(pc_high ^ ghist(xorBits - 1, 0), pc_low)
  }

  // 修复返回类型为 (Bool, UInt)
  def get_predict(uop: MicroOp): (Bool, UInt) = {
    val l1_val = l1_table(get_l1_idx(uop.debug_pc))
    val l2_val = l2_table(get_l2_idx(uop.debug_pc, uop.mdp_ghist))
    val l3_val = l3_table(get_l3_idx(uop.debug_pc, uop.mdp_ghist))

    // 优先最高级历史
    val wait_state = Wire(Bool())
    val st_dist = Wire(UInt(3.W))

    when(l3_val === 1.U) {
      wait_state := true.B // 修复为 Bool 赋值
      st_dist := 0.U
    }.elsewhen(l2_val =/= 0.U) {
      wait_state := true.B
      st_dist := l2_val
    }.elsewhen(l1_val =/= 0.U) {
      wait_state := true.B
      st_dist := l1_val
    }.otherwise {
      wait_state := false.B
      st_dist := 0.U
    }
    
    (wait_state, st_dist)
  }

  def update(uop: MicroOp, conflict_dist: UInt): Unit = {
    val l1_idx = get_l1_idx(uop.debug_pc)
    val l2_idx = get_l2_idx(uop.debug_pc, uop.mdp_ghist)
    val l3_idx = get_l3_idx(uop.debug_pc, uop.mdp_ghist)

    val l1_val = l1_table(l1_idx)
    val l2_val = l2_table(l2_idx)

    // 不一致时进行表级提升
    when(l1_val === 0.U) {
      l1_table(l1_idx) := conflict_dist
    }.elsewhen(l1_val =/= conflict_dist && l2_val === 0.U) {
      l2_table(l2_idx) := conflict_dist
    }.elsewhen(l2_val =/= 0.U && l2_val =/= conflict_dist) {
      l3_table(l3_idx) := 1.U
    }
  }
}