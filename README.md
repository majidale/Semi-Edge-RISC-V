# Semi-Edge-RISC-V

Welcome to the official **Semi Edge RISC-V** repository!  
This repo contains all the **helping documents, notes, codes, and resources** used in my RISC-V tutorials and YouTube videos.  
If you’re learning RISC-V step by step from my channel, this is the right place to follow along.  

---

## 📂 Repository Contents
- 📘 Documentation & Notes  
- 📝 RISC-V Instruction References  
- 💻 Verilog/SystemVerilog Codes  
- 🔧 Testbenches & Examples  
- 🖥️ Diagrams & Slides  

---

## ▶️ YouTube Playlist
Watch the complete **RISC-V Tutorial Series** on the [Semi Edge YouTube Channel](https://www.youtube.com/@semiedge1).  

---
## Single Cycle Datapath
<p align="center">
  <img src="https://github.com/majidale/Semi-Edge-RISC-V/blob/main/Single%20Cycle%20Datapath.png" alt="Instruction Format" width="600">
</p>

---

##  Instruction Format
<p align="center">
  <img src="https://github.com/majidale/Semi-Edge-RISC-V/blob/main/Instruction%20Format.png" alt="Instruction Format" width="600">
</p>

---
## Supported Instructions
This design implements all 47 RV32I instructions:
**R-type:** add, sub, sll, slt, sltu, xor, srl, sra, or, and
**I-type (arithmetic):** addi, slti, sltiu, xori, ori, andi, slli, srli, srai
**Load:** lb, lh, lw, lbu, lhu
**Store:** sb, sh, sw
**Branch:** beq, bne, blt, bge, bltu, bgeu
**Jump:** jal, jalr
**Upper Immediate:** lui, auipc

---
##  Simulation Results
<p align="center">
  <img src="https://github.com/majidale/Semi-Edge-RISC-V/blob/main/RISCV_Single_Cycle/Simulation%20Results.png" >
</p>
---
