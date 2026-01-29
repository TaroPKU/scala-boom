package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.Str

import boom.common._

object MDP_GHistory_length {
  val length = 8
}

case class LWTConfig(
  numEntries: Int = 1024,
  predictionWidth: Int = 1,
  pcStartBit: Int = 1,
  clearCyclePeriod: Int = 2097152
) {
  require(isPow2(numEntries), "表大小必须是2的幂")
  require(clearCyclePeriod > 0, "清零周期必须大于0")

  val indexBits: Int = log2Ceil(numEntries)
  val pcEndBit: Int = pcStartBit + indexBits - 1
  val clearCycleBits: Int = log2Ceil(clearCyclePeriod)

  private val setValueBigInt: BigInt = (BigInt(1) << predictionWidth) - 1
  val setValue: UInt = setValueBigInt.U(predictionWidth.W)

  def extractIndex(pc: UInt): UInt = {
    require(pc.getWidth > pcEndBit, s"PC宽度${pc.getWidth}不够，需要至少${pcEndBit + 1}位")
    pc(pcEndBit, pcStartBit)
  }
}

class LWT_Entry(val predictWidth: Int = 1)(implicit p: Parameters) extends BoomBundle()(p)
{
    val predict = UInt(predictWidth.W)
}

sealed trait MDPKind
object MDPKind {
  case object LWT extends MDPKind
  case object CTX_MDP extends MDPKind
}

case class MDPParams(
  kind: MDPKind,
  lwt: LWTConfig = LWTConfig(),
  ctxResetCyclePeriod: Option[Int] = None
)
object MDPParams {
  def lwt(config: LWTConfig = LWTConfig()): MDPParams = MDPParams(MDPKind.LWT, config)
  def ctx_mdp(config: LWTConfig = LWTConfig(), ctxResetCyclePeriod: Option[Int] = None): MDPParams =
    MDPParams(MDPKind.CTX_MDP, config, ctxResetCyclePeriod)
}

object MDP {
  def apply(params: MDPParams)(implicit p: Parameters): MDP = new MDP(params)
}

/**
  * MDP 预测器封装：目前支持 LWT（PC 索引的预测位表），并内建“周期清零”的计数器。
  *
  * 设计目标：
  * - LSU 内只通过 get_predict/update/mdp_reset 使用
  * - 参数集中在本文件，后续扩展更多 predictor 时只新增 kind 分支
  */
final class MDP(params: MDPParams)(implicit p: Parameters) {
  private val lwtConfig = params.lwt

  private val hasCtx = params.kind match {
    case MDPKind.CTX_MDP => true
    case _               => false
  }

  private val ctxNumEntries: Int = 1 << MDP_GHistory_length.length
  private val ctxResetPeriod: Int = {
    val defaultRaw = (lwtConfig.clearCyclePeriod / 3) - 7
    val default = math.max(131072, defaultRaw)
    params.ctxResetCyclePeriod.getOrElse(default)
  }
  require(!hasCtx || ctxResetPeriod > 0, "CTX_MDP 清零周期必须大于0")

  private val mdp_lwt = RegInit(
    0.U.asTypeOf(Vec(lwtConfig.numEntries, new LWT_Entry(lwtConfig.predictionWidth)))
  )

  private val lwtClearCounter = RegInit(0.U(lwtConfig.clearCycleBits.W))
  private val lwtShouldClear = lwtClearCounter === (lwtConfig.clearCyclePeriod - 1).U
  lwtClearCounter := Mux(lwtShouldClear, 0.U, lwtClearCounter + 1.U)

  private val mdp_ctx = if (hasCtx) {
    Some(RegInit(VecInit(Seq.fill(ctxNumEntries)(false.B))))
  } else {
    None
  }

  private val ctxClearCounter = if (hasCtx) {
    Some(RegInit(0.U(log2Ceil(ctxResetPeriod).W)))
  } else {
    None
  }
  private val ctxShouldClear: Bool = if (hasCtx) {
    val ctr = ctxClearCounter.get
    val should = ctr === (ctxResetPeriod - 1).U
    ctr := Mux(should, 0.U, ctr + 1.U)
    should
  } else {
    false.B
  }

  private def reset_lwt(cond: Bool): Unit = {
    when(cond) {
      for (i <- 0 until lwtConfig.numEntries) {
        mdp_lwt(i).predict := 0.U
      }
    }
  }

  private def reset_ctx(cond: Bool): Unit = {
    if (hasCtx) {
      when(cond) {
        for (i <- 0 until ctxNumEntries) {
          mdp_ctx.get(i) := false.B
        }
      }
    }
  }

  /**
    * 重置预测表。
    * - LWT：由 lwtCond 控制
    * - CTX_MDP 附加表：由 ctxCond 控制（仅在 CTX_MDP 模式下生效）
    */
  def mdp_reset(lwtCond: Bool = true.B, ctxCond: Bool = true.B): Unit = {
    reset_lwt(lwtCond)
    reset_ctx(ctxCond)
  }

  // 周期清零：LWT 与 CTX 表可独立配置周期
  mdp_reset(lwtCond = lwtShouldClear, ctxCond = ctxShouldClear)

  private def ctx_index(uop: MicroOp): UInt = {
    val pc_part = uop.debug_pc(MDP_GHistory_length.length, 1)
    pc_part ^ uop.mdp_ghist
  }

  /** 查询 uop 的预测结果（true 表示需要 mdp_wait）。 */
  def get_predict(uop: MicroOp): Bool = {
    params.kind match {
      case MDPKind.LWT =>
        mdp_lwt(lwtConfig.extractIndex(uop.debug_pc)).predict.orR
      case MDPKind.CTX_MDP =>
        val lwt_p = mdp_lwt(lwtConfig.extractIndex(uop.debug_pc)).predict.orR
        val ctx_p = mdp_ctx.get(ctx_index(uop))
        lwt_p || ctx_p
    }
  }

  /** 当 en 为真时更新 uop 对应的预测表项（默认更新）。 */
  def update(uop: MicroOp, en: Bool = true.B): Unit = {
    params.kind match {
      case MDPKind.LWT =>
        when(en) {
          mdp_lwt(lwtConfig.extractIndex(uop.debug_pc)).predict := lwtConfig.setValue
        }
      case MDPKind.CTX_MDP =>
        when(en) {
          mdp_lwt(lwtConfig.extractIndex(uop.debug_pc)).predict := lwtConfig.setValue
          mdp_ctx.get(ctx_index(uop)) := true.B
        }
    }
  }
}