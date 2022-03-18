/********************************** (C) COPYRIGHT *******************************
* File Name          : macros.h
* Author             : WCH
* Version            : V1.0
* Date               : 2021/07/31
* Description        : 某些调试用的宏
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#define caller_gpr_store_to_array( array ) \
  asm("addi sp, sp, -8"); \
  asm("sw x9, 4(sp)"); \
  asm("la x9," #array); \
  asm("sw x1, 0(x9)"); \
  asm("sw x5, 4(x9)"); \
  asm("sw x6, 8(x9)"); \
  asm("sw x7, 12(x9)"); \
  asm("sw x10, 16(x9)"); \
  asm("sw x11, 20(x9)"); \
  asm("sw x12, 24(x9)"); \
  asm("sw x13, 28(x9)"); \
  asm("sw x14, 32(x9)"); \
  asm("sw x15, 36(x9)"); \
  asm("sw x16, 40(x9)"); \
  asm("sw x17, 44(x9)"); \
  asm("sw x28, 48(x9)"); \
  asm("sw x29, 52(x9)"); \
  asm("sw x30, 56(x9)"); \
  asm("sw x31, 60(x9)"); \
  asm("lw x9,  4(sp)"); \
  asm("addi sp, sp, 8");

#define caller_gpr_load_from_array( array ) \
  asm("addi sp, sp, -8"); \
  asm("sw x9, 4(sp)"); \
  asm("la x9," #array); \
  asm("lw x1, 0(x9)"); \
  asm("lw x5, 4(x9)"); \
  asm("lw x6, 8(x9)"); \
  asm("lw x7, 12(x9)"); \
  asm("lw x10, 16(x9)"); \
  asm("lw x11, 20(x9)"); \
  asm("lw x12, 24(x9)"); \
  asm("lw x13, 28(x9)"); \
  asm("lw x14, 32(x9)"); \
  asm("lw x15, 36(x9)"); \
  asm("lw x16, 40(x9)"); \
  asm("lw x17, 44(x9)"); \
  asm("lw x28, 48(x9)"); \
  asm("lw x29, 52(x9)"); \
  asm("lw x30, 56(x9)"); \
  asm("lw x31, 60(x9)"); \
  asm("lw x9,  4(sp)"); \
  asm("addi sp, sp, 8");

#define caller_gpr_write_to_imm( imm ) \
  asm("li x1,  " #imm ); \
  asm("li x5,  " #imm ); \
  asm("li x6,  " #imm ); \
  asm("li x7,  " #imm ); \
  asm("li x10, " #imm ); \
  asm("li x11, " #imm ); \
  asm("li x12, " #imm ); \
  asm("li x13, " #imm ); \
  asm("li x14, " #imm ); \
  asm("li x15, " #imm ); \
  asm("li x16, " #imm ); \
  asm("li x17, " #imm ); \
  asm("li x28, " #imm ); \
  asm("li x29, " #imm ); \
  asm("li x30, " #imm ); \
  asm("li x31, " #imm );


#define exception_next_pc() \
       asm(" csrr  t0,  mepc");\
       asm(" lb    a0,  0(t0)");\
       asm("li    a1,  0x03");\
       asm("and   a0,  a0,  a1");\
       asm("bne   a0,  a1,  1f");\
       asm("addi  t0, t0, 2");\
       asm("1:    addi  t0, t0, 2");\
       asm("csrw  mepc,  t0");

//csr option
#define read_csr(reg) ({ unsigned long __tmp;                               \
    asm volatile ("csrr %0, " #reg : "=r"(__tmp));                          \
        __tmp; })

#define write_csr(reg, val) ({                                              \
    if (__builtin_constant_p(val) && (unsigned long)(val) < 32)             \
        asm volatile ("csrw " #reg ", %0" :: "i"(val));                     \
    else                                                                    \
        asm volatile ("csrw " #reg ", %0" :: "r"(val)); })


#define set_csr(reg, bit) ({ unsigned long __tmp;                           \
    if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32)             \
        asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit));   \
    else                                                                    \
        asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit));   \
            __tmp; })

#define clear_csr(reg, bit) ({ unsigned long __tmp;                         \
    if (__builtin_constant_p(bit) && (unsigned long)(bit) < 32)             \
        asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "i"(bit));   \
    else                                                                    \
        asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "r"(bit));   \
            __tmp; })








