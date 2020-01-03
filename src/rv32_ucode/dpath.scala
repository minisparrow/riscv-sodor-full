//**************************************************************************
// RISCV U-Coded Processor Data Path
//--------------------------------------------------------------------------
//
// Christopher Celio
// 2011 May 28

package Sodor
{

import chisel3._
import chisel3.util.{Cat, Fill, MuxLookup, MuxCase}

import Constants._
import Common.{MemPortIo, SodorConfiguration, Str, CSRFile, CSR, Causes}
import Common.Constants._


class DatToCtlIo extends Bundle() 
{
   val inst     = Output(UInt(32.W))
   val alu_zero = Output(Bool())
   val xcpt = Output(Bool())
   val ma_str = Output(Bool())
}


class DpathIo(implicit val conf: SodorConfiguration) extends Bundle() 
{
   val mem  = new MemPortIo(conf.xprlen)
   val ctl  = Flipped(new CtlToDatIo())
   val dat  = new DatToCtlIo()
}  


class DatPath(implicit val conf: SodorConfiguration) extends Module
{
   val io = IO(new DpathIo())
   io := DontCare


   // forward declarations
   val xcpt      = Reg(Bool())
   val imm       = Wire(UInt(conf.xprlen.W))
   val alu       = Wire(UInt(conf.xprlen.W))
   val reg_rdata = Wire(UInt(conf.xprlen.W))
   val csr_rdata = Wire(UInt(conf.xprlen.W))
   val exception_target = Wire(UInt(conf.xprlen.W))

   // The Bus 
   // (this is a bus-based RISCV implementation, so all data movement goes
   // across this wire)
   val bus = MuxCase(0.U, Array(
               (io.ctl.en_imm)                  -> imm(conf.xprlen-1,0),
               (io.ctl.en_alu)                  -> alu(conf.xprlen-1,0), 
               (io.ctl.en_reg & ~io.ctl.reg_wr & 
                 (io.ctl.reg_sel =/= RS_CR))     -> reg_rdata(conf.xprlen-1,0),
               (io.ctl.en_mem & ~io.ctl.mem_wr) -> io.mem.resp.bits.data(conf.xprlen-1,0),
               (io.ctl.en_reg & ~io.ctl.reg_wr & 
                  (io.ctl.reg_sel === RS_CR))   -> csr_rdata
             ))
 
   

   // IR Register
   val ir    = RegInit(0.U(conf.xprlen.W))
   when (io.ctl.ld_ir) { ir := bus }
   io.dat.inst := ir
    
   // A Register
   val reg_a = RegInit("haaaa".U(conf.xprlen.W))
   when (io.ctl.ld_a) { reg_a := bus }
     
   // B Register
   val reg_b = RegInit("hbbbb".U(conf.xprlen.W))
   when (io.ctl.ld_b) { reg_b := bus }
    
   // MA Register
   val reg_ma  = RegInit("heeee".U(conf.xprlen.W))
   when (io.ctl.ld_ma) { reg_ma := bus }

   // IR Immediate
   imm := MuxCase(0.U, Array(
             (io.ctl.is_sel === IS_I)  -> Cat(Fill(20,ir(31)),ir(31,20)), 
             (io.ctl.is_sel === IS_S)  -> Cat(Fill(20,ir(31)),ir(31,25),ir(11,7)),
             (io.ctl.is_sel === IS_U)  -> Cat(ir(31,12),0.S(12.W)),
             (io.ctl.is_sel === IS_B)  -> Cat(Fill(20,ir(31)),ir(7),ir(30,25),ir(11,8),0.U(1.W)),
             (io.ctl.is_sel === IS_J)  -> Cat(Fill(20,ir(31)),ir(19,12),ir(20),ir(30,21),0.U(1.W)),
             (io.ctl.is_sel === IS_Z)  -> Cat(0.U(27.W), ir(19,15))
           ))

     

   
   // Register File (Single Port)
   // also holds the PC register
   val rs1 = ir(RS1_MSB, RS1_LSB)
   val rs2 = ir(RS2_MSB, RS2_LSB)
   val rd  = ir(RD_MSB,  RD_LSB)

   val reg_addr  = MuxCase(0.U, Array(
                     (io.ctl.reg_sel === RS_PC)  -> PC_IDX,
                     (io.ctl.reg_sel === RS_RD)  -> rd,
                     (io.ctl.reg_sel === RS_RS1) -> rs1,
                     (io.ctl.reg_sel === RS_RS2) -> rs2
                   ))
 
   //note: I could be far more clever and save myself on wasted registers here...
   //32 x-registers, 1 pc-register
   val regfile = Reg(Vec(33, UInt(32.W)))

   when (io.ctl.en_reg & !io.dat.xcpt & io.ctl.reg_wr & reg_addr(4,0) =/= 0.U)
   {
      regfile(reg_addr) := bus
   } .elsewhen (io.ctl.en_reg & !io.dat.xcpt & io.ctl.reg_wr & reg_addr(5) === 1.U) {
      regfile(reg_addr) := Cat(bus(31,1),0.U(1.W)) // PC
   }

   // This is a hack to make it look like the CSRFile is part of the regfile
   reg_rdata :=  MuxCase(regfile(reg_addr), Array(
                    (io.ctl.reg_sel === RS_CR) -> csr_rdata,
                    (reg_addr === 0.U)     -> 0.asUInt(conf.xprlen.W)))
                    
   // CSR addr Register
   val csr_addr = RegInit(0.U(12.W))
   when(io.ctl.reg_wr & (io.ctl.reg_sel === RS_CA)) {
     csr_addr := bus
   }

   val csr_wdata = RegInit(0.U(conf.xprlen.W))
   when(io.ctl.reg_wr & (io.ctl.reg_sel === RS_CR)) {
     csr_wdata := bus
   }
   
   // Control Status Registers
   val csr = Module(new CSRFile())
   csr.io := DontCare
   csr.io.rw.addr  := csr_addr
   csr.io.rw.wdata := csr_wdata
   csr.io.rw.cmd   := io.ctl.csr_cmd
   csr_rdata       := csr.io.rw.rdata 
   csr.io.retire    := io.ctl.upc_is_fetch
   // illegal micro-code encountered
   csr.io.pc        := regfile(PC_IDX) - 4.U 
   exception_target := csr.io.evec

   // LOGIC to check for misaligned address 
   val ma_load          = Wire(Bool())
   val ma_str           = Wire(Bool())
   val ma_jump          = Wire(Bool())
   val rollback_reg     = Reg(UInt(conf.xprlen.W))
   val rollback_idx     = Reg(UInt(5.W))
   val rollback         = RegInit(false.B)
   val tval             = Reg(UInt(conf.xprlen.W))
   val cause            = Reg(UInt(conf.xprlen.W))
   val ls_addr_ma_valid = MuxLookup(io.ctl.msk_sel(1,0) ,false.B, Array( 2.U -> reg_ma(0), 3.U -> reg_ma(1,0).orR ))
   ma_jump      := alu(1) && io.ctl.en_reg & io.ctl.reg_wr & (io.ctl.reg_sel === RS_PC) & (io.ctl.ubr === UBR_J)
   ma_load      := !io.ctl.mem_wr && io.ctl.en_mem && ls_addr_ma_valid
   ma_str       := io.ctl.mem_wr && io.ctl.en_mem && ls_addr_ma_valid
   csr.io.xcpt  := xcpt 
   when (ma_load || ma_str || ma_jump || io.ctl.illegal) {
      cause := MuxCase(0.U, Array(
                        ma_jump -> Causes.misaligned_fetch.U,
                        ma_load -> Causes.misaligned_load.U,
                        ma_str  -> Causes.misaligned_store.U,
                        io.ctl.illegal -> Causes.illegal_instruction.U ))
      tval  := MuxCase(0.U, Array(
                        ma_jump -> Cat(bus(31,1),0.U(1.W)),
                        ma_load -> reg_ma,
                        ma_str  -> reg_ma,
                        io.ctl.illegal -> ir ))
      xcpt  := !io.ctl.illegal
   }
   when (io.ctl.alu_op === ALU_EVEC) {
      xcpt := false.B
   }
   csr.io.xcpt    := xcpt || RegNext(io.ctl.illegal)
   csr.io.cause   := cause 
   csr.io.tval    := tval
   io.dat.xcpt    := ma_load || ma_str || ma_jump 
   io.dat.ma_str  := ma_str
   when ((io.ctl.upc_is_jalr_wb || io.ctl.upc_is_jal_wb) && reg_addr.orR) {
      rollback_reg := regfile(reg_addr)
      rollback_idx := reg_addr
      rollback := true.B
   }
   when (ma_jump && rollback) {
      regfile(rollback_idx) := rollback_reg
      rollback := false.B
   }

   // Add your own uarch counters here!
   csr.io.counters.foreach(_.inc := false.B)

   // ALU
   val alu_shamt = reg_b(4,0).asUInt

   alu := MuxCase(0.U, Array[(Bool, UInt)](
              (io.ctl.alu_op === ALU_COPY_A)  ->  reg_a,
              (io.ctl.alu_op === ALU_COPY_B)  ->  reg_b,
              (io.ctl.alu_op === ALU_INC_A_1) ->  (reg_a  +  1.U),
              (io.ctl.alu_op === ALU_DEC_A_1) ->  (reg_a  -  1.U),
              (io.ctl.alu_op === ALU_INC_A_4) ->  (reg_a  +  4.U),
              (io.ctl.alu_op === ALU_DEC_A_4) ->  (reg_a  -  4.U),
              (io.ctl.alu_op === ALU_ADD)     ->  (reg_a  +  reg_b),
              (io.ctl.alu_op === ALU_SUB)     ->  (reg_a  -  reg_b),
              (io.ctl.alu_op === ALU_SLL)     -> ((reg_a << alu_shamt)(conf.xprlen-1,0)),
              (io.ctl.alu_op === ALU_SRL)     ->  (reg_a >> alu_shamt),
              (io.ctl.alu_op === ALU_SRA)     ->  (reg_a.asSInt >> alu_shamt).asUInt,
              (io.ctl.alu_op === ALU_AND)     ->  (reg_a & reg_b),
              (io.ctl.alu_op === ALU_OR)      ->  (reg_a | reg_b),
              (io.ctl.alu_op === ALU_XOR)     ->  (reg_a ^ reg_b),
              (io.ctl.alu_op === ALU_SLT)     ->  (reg_a.asSInt < reg_b.asSInt).asUInt,
              (io.ctl.alu_op === ALU_SLTU)    ->  (reg_a < reg_b),
              (io.ctl.alu_op === ALU_INIT_PC) ->  START_ADDR,
              (io.ctl.alu_op === ALU_MASK_12) ->  (reg_a & ~((1<<12)-1).asUInt(conf.xprlen.W)),
              (io.ctl.alu_op === ALU_EVEC)    ->  exception_target
            ))
  
   // Output Signals to the Control Path
   io.dat.alu_zero := (alu === 0.U)
   
   // Output Signals to the Memory
   io.mem.req.bits.addr := reg_ma
   io.mem.req.bits.data := bus
   // Retired Instruction Counter 
   val irt_reg = RegInit(0.U(conf.xprlen.W))
   when (io.ctl.upc_is_fetch) { irt_reg := irt_reg + 1.U }

   // Printout
   printf("%cCyc= %d (MA=0x%x) %d %c %c RegAddr=%d Bus=0x%x A=0x%x B=0x%x PCReg=( 0x%x ) UPC=%d InstReg=[ 0x%x : DASM(%x) ]\n"
      , Mux(io.ctl.upc_is_fetch, Str("F"), Str(" "))
      , csr.io.time(31,0)
      , reg_ma
      , io.ctl.reg_sel
      , Mux(io.ctl.en_mem, Str("E"), Str(" ")) 
      , Mux(io.ctl.illegal, Str("X"), Str(" ")) 
      , reg_addr
      , bus
      , reg_a
      , reg_b
      , regfile(PC_IDX) // this is the PC register
      , io.ctl.upc
      , ir
      , ir
      );

}

}

