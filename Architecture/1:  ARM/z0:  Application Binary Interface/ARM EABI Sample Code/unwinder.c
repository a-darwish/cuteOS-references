/* unwinder.c
 *
 * Copyright 2002-2007 ARM Limited. All rights reserved.
 *
 * Your rights to use this code are set out in the accompanying licence
 * text file LICENCE.txt (ARM contract number LEC-ELA-00080 v2.0).
 */
/*
 * RCS $Revision: 170787 $
 * Checkin $Date: 2011-09-07 10:57:11 +0100 (Wed, 07 Sep 2011) $
 * Revising $Author: pwright $
 */

/* Language-independent unwinder implementation */

/* This source file is compiled automatically by ARM's make system into
 * multiple object files. The source regions constituting object file
 * xxx.o are delimited by ifdef xxx_c / endif directives.
 *
 * The source regions currently marked are:
 * unwinder_c
 * unwind_delete_c
 * unwind_activity_c
 */

#include <stddef.h>
#include <stdlib.h>
/* Environment: */
#include "unwind_env.h"
/* Language-independent unwinder declarations: */
#include "unwinder.h"

/* Define UNWIND_ACTIVITY_DIAGNOSTICS for printed information from _Unwind_Activity */
/* Define VRS_DIAGNOSTICS for printed diagnostics about VRS operations */

#if defined(VRS_DIAGNOSTICS) || defined(UNWIND_ACTIVITY_DIAGNOSTICS)
extern int printf(const char *, ...);
#endif

/* =========================                      ========================= */
/* =========================      UCB usage       ========================= */
/* =========================                      ========================= */

/* This implementation uses the UCB unwinder_cache as follows:
 * reserved1 is documented in the EABI as requiring initialisation to 0.
 *  It is used to manage nested simultaneous propagation. If the value is 0,
 *  the UCB is participating in no propagations. If the value is 1, the UCB
 *  is participating in one propagation. Otherwise the value is a pointer to
 *  a structure holding saved UCB state from the next propagation out.
 *  The structure used is simply a mallocated UCB.
 * reserved2 is used to preserve the call-site address over calls to a
 *  personality routine and cleanup.
 * reserved3 is used to cache the PR address.
 */

#define NESTED_CONTEXT      unwinder_cache.reserved1
#define SAVED_CALLSITE_ADDR unwinder_cache.reserved2
#define PR_ADDR             unwinder_cache.reserved3


/* =========================                      ========================= */
/* ========================= Virtual register set ========================= */
/* =========================                      ========================= */

/* The approach taken by this implementation is to use the real machine
 * registers to hold all but the values of core (integer)
 * registers. Consequently the implementation must use only the core
 * registers except when manipulating the virtual register set. Non-core
 * registers are saved only on first use, so the single implementation can
 * cope with execution on processors which lack certain registers.  The
 * registers as they were at the start of the propagation must be preserved
 * over phase 1 so that the machine state is correct at the start of phase
 * 2. This requires a copy to be taken (which can be stack allocated). During
 * a stack unwind (phase 1 or phase 2), the "current" virtual register set is
 * implemented as core register values held in a data structure, and non-core
 * register values held in the registers themselves. To ensure that all
 * original register values are available at the beginning of phase 2, the
 * core registers are saved in a second structure at the start of phase 1 and
 * the non-core registers are demand-saved into another part of the data
 * structure that holds the current core registers during the phase 1 stack
 * unwind.
 */
/* Extent to which the access routines are implemented:
 * _Unwind_VRS_Get and _Unwind_VRS_Set implement only access to the core registers.
 * _Unwind_VRS_Pop implements only popping of core and vfp registers.
 * There is no support here for the Intel WMMX registers, but space is nevertheless
 * reserved in the virtual register set structure to indicate whether demand-saving
 * of those registers is required (as they are unsupported, it never is). The space
 * costs nothing as it is required for alignment.
 * The level of supported functionality is compliant with the requirements of the
 * Exceptions ABI.
 */

typedef unsigned char bool;
struct core_s  { uint32_t r[16]; };        /* core integer regs */
struct vfp_s   { uint64_t d[32]; };        /* VFP registers saved in FSTMD format */

/* Phase 1 virtual register set includes demand-save areas */
/* The phase 2 virtual register set must be a prefix of the phase 1 set */
typedef struct phase1_virtual_register_set_s {
  /* demand_save flag == 1 means save the registers in the demand-save area */
  bool demand_save_vfp_low;
  bool demand_save_vfp_high;
  bool demand_save_wmmxd;
  bool demand_save_wmmxc;
  struct core_s core;      /* current core registers */
  struct vfp_s  vfp;       /* demand-saved vfp registers D0-D31 */
} phase1_virtual_register_set;

/* Phase 2 virtual register set has no demand-save areas */
/* The phase 2 virtual register set must be a prefix of the phase 1 set */
/* The assembly fragments for _Unwind_RaiseException and _Unwind_Resume create
 * a phase2_virtual_register_set_s by hand so be careful.
 */
typedef struct phase2_virtual_register_set_s {
  /* demand_save flag == 1 means save the registers in the demand-save area */
  /* Always 0 in phase 2 */
  bool demand_save_vfp_low;
  bool demand_save_vfp_high;
  bool demand_save_wmmxd;
  bool demand_save_wmmxc;
  struct core_s core;      /* current core registers */
} phase2_virtual_register_set;


/* Helper macros for the embedded assembly.
 * Note that these are specific to the ARM compiler,
 * and the earliest target is __TARGET_ARCH_4.
 */

#if defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_6_M) || defined(__TARGET_ARCH_6S_M)
  #define HAS_ARM 0
#else
  #define HAS_ARM 1
#endif

#if (__TARGET_ARCH_THUMB < 4) || defined(__TARGET_ARCH_6_M) || defined(__TARGET_ARCH_6S_M)
  #define HAS_THUMB2 0
#else
  #define HAS_THUMB2 1
#endif

#if HAS_ARM || HAS_THUMB2
  #define HAS_THUMB1_ONLY 0
#else
  #define HAS_THUMB1_ONLY 1
#endif

#if HAS_THUMB1_ONLY
  #define SUPPORT_VFP 0
#else
  #define SUPPORT_VFP 1
#endif

#if defined(__APCS_INTERWORK) && \
    (defined(__TARGET_ARCH_4) || defined(__TARGET_ARCH_4T) || defined(__TARGET_ARCH_5))
  #define OLD_STYLE_INTERWORKING 1
#else
  #define OLD_STYLE_INTERWORKING 0
#endif

#if defined(__TARGET_ARCH_4)
  #define HAVE_BX 0
#else
  #define HAVE_BX 1
#endif

#if HAVE_BX
  #define RET_LR bx lr
#else
  #define RET_LR mov pc,lr
#endif

#if HAS_ARM && defined(__thumb)
#define MAYBE_SWITCH_TO_ARM_STATE SWITCH_TO_ARM_STATE
#define MAYBE_CODE16 code16
#else
#define MAYBE_SWITCH_TO_ARM_STATE /* nothing */
#define MAYBE_CODE16              /* nothing */
#endif

/* Private declarations */

extern NORETURNDECL void __ARM_unwind_next_frame(_Unwind_Control_Block *ucbp,
                                                 phase2_virtual_register_set *vrsp);

#ifdef unwinder_c

/* ----- Helper routines, private ----- */

/* R_ARM_PREL31 is a place-relative 31-bit signed relocation.  The
 * routine takes the address of a location that was relocated by
 * R_ARM_PREL31, and returns an absolute address.
 */
static FORCEINLINE uint32_t __ARM_resolve_prel31(void *p)
{
  return (uint32_t)((((*(int32_t *)p) << 1) >> 1) + (int32_t)p);
}

/* ----- Helper routines, private but external ----- */

/* Note '%0' refers to local label '0' */

#if SUPPORT_VFP

__asm void __ARM_Unwind_VRS_VFPpreserve_low(void *vfpp)
{
vfp_d0 CN 0;
  /* Preserve the low vfp registers in the passed memory */
#if HAS_ARM && defined(__thumb)
  macro;
  SWITCH_TO_ARM_STATE;
1
  align 4;
2
  assert (%2 - %1) = 0;
  bx pc;
  nop;
  code32;
  mend;
#endif

  MAYBE_SWITCH_TO_ARM_STATE;
  stc   p11,vfp_d0,[r0],{0x20};  /* 0xec800b20  FSTMIAD r0,{d0-d15} */
  RET_LR;
  MAYBE_CODE16;
}

__asm void __ARM_Unwind_VRS_VFPpreserve_high(void *vfpp)
{
vfp_d16 CN 0;                      /* =16 when used with stcl */
  /* Preserve the high vfp registers in the passed memory */
  MAYBE_SWITCH_TO_ARM_STATE;
  stcl  p11,vfp_d16,[r0],{0x20};  /* 0xecc00b20  FSTMIAD r0,{d16-d31} */
  RET_LR;
  MAYBE_CODE16;
}

__asm void __ARM_Unwind_VRS_VFPrestore_low(void *vfpp)
{
  /* Restore the low vfp registers from the passed memory */
vfp_d0 CN 0;
  MAYBE_SWITCH_TO_ARM_STATE;
  ldc   p11,vfp_d0,[r0],{0x20};  /* 0xec900b20  FLDMIAD r0,{d0-d15} */
  RET_LR;
  MAYBE_CODE16;
}

__asm void __ARM_Unwind_VRS_VFPrestore_high(void *vfpp)
{
  /* Restore the high vfp registers from the passed memory */
vfp_d16 CN 0;                      /* =16 when used with ldcl */
  MAYBE_SWITCH_TO_ARM_STATE;
  ldcl   p11,vfp_d16,[r0],{0x20};  /* 0xecd00b20  FLDMIAD r0,{d16-d31} */
  RET_LR;
  MAYBE_CODE16;
}

#endif /* SUPPORT_VFP */

__asm NORETURNDECL void __ARM_Unwind_VRS_corerestore(void *corep)
{
  preserve8;

#if HAS_THUMB1_ONLY
    /*
     * Restoring all registers in Thumb-1 is rather tricky. The
     * method I'm going to use is:
     * 
     *  - We will inevitably need some low registers to use as
     *    temporaries.
     * 
     *  - Therefore, we prepare a small memory block just below the
     *    _target_ sp value, containing the target PC and the
     *    target values of those low registers.
     * 
     *  - We load all the other target register values out of corep
     *    into their registers.
     * 
     *  - Then we set sp to point at the bottom of the
     *    abovementioned memory block, i.e. at (target sp) minus
     *    (size of block). We make sure the block contains an even
     *    number of words, so this step preserves stack alignment.
     * 
     *  - Finally, we pop the corrupted low registers plus pc,
     *    which completes the register restore.
     * 
     * Assuming that the target sp is lower than the temporary sp
     * value (which we can, because the corep block itself will be
     * stored on the stack below that point), this is
     * interrupt-safe because at no instant is valuable data stored
     * below the current value of sp. The only thing that
     * potentially worries me is that our temporary memory block
     * might overlap corep. To protect against this we must ensure
     * we write the temporary block in a downward direction and
     * read every potentially overwritten value before we do
     * anything which might overwrite it.
     */
    ldr r1, [r0, #13*4];  /* get target sp */
    subs r1, r1, #4*4;    /* temporary block is 4 registers long */
    ldr r2, [r0, #15*4];  /* get target pc */
    str r2, [r1, #3*4];   /* and store it at top of temp block */
    ldr r2, [r0, #14*4];  /* read target lr before we risk overwriting it */
    mov r14, r2;          /* and put it in r14 proper */
    ldr r2, [r0, #2*4];   /* now get target r2 */
    str r2, [r1, #2*4];   /* and put it in temp block */
    ldr r2, [r0, #1*4];   /* same with target r1, which is already safe */
    str r2, [r1, #1*4];   /*   because we've already got r1 == targetsp-16 */
    ldr r2, [r0, #12*4];  /* read target ip before overwriting it */
    mov r12, r2;          /* and put it in real ip */
    ldr r2, [r0, #0*4];   /* now get target r0 */
    str r2, [r1, #0*4];   /* and put it in temp block, which is now complete */
    /* Now our temp block is complete, and r1 points at it. Next we must
     * restore all the registers from r3 to r11 inclusive, using r2 as a
     * scratch register. */
    ldr r2, [r0, #11*4];
    mov r11, r2;
    ldr r2, [r0, #10*4];
    mov r10, r2;
    ldr r2, [r0, #9*4];
    mov r9, r2;
    ldr r2, [r0, #8*4];
    mov r8, r2;
    movs r2, r0;
    adds r2, r2, #3*4;
    ldmia r2!, {r3-r7};
    /* Now all registers are in their target state except r0-r2 and pc,
     * so we restore those by popping them out of the temp block. */
    mov r13, r1;
    pop {r0-r2,r15};

#elif HAS_THUMB2 && defined(__thumb)
  /* Make use of a small memory block just below the target sp value */
  mov   r13, r0;
  ldr   r0, [r13, #13*4];  /* the sp to restore */
  ldr   r14, [r13, #14*4]; /* restore lr */
  ldr   r3, [r13, #15*4];  /* the pc to restore */
  ldr   r2, [r13], #4;     /* pop the r0 to restore ... */
  stmdb r0!, {r2,r3};      /* push at sp to restore; at worst overwrite saved lr and pc */
  ldmia r13, {r1-r12};     /* after which we have r0, sp, pc to restore */
  mov   r13, r0;           /* now on the final stack with 2 values to pop */
  pop   {r0,r15};
#else /* In ARM state, or Thumb1 with ARM available */
  MAYBE_SWITCH_TO_ARM_STATE;
  /* Make use of a small memory block just below the target sp value */
#if OLD_STYLE_INTERWORKING
  mov   r13, r0;
  ldr   r0, [r13, #13*4];  /* the sp to restore */
  ldr   r14, [r13, #14*4]; /* restore lr */
  ldr   r1, [r13, #15*4];  /* the pc to restore */
  tst   r1, #1;            /* EQ if return to ARM, NE for return to Thumb */
  str   r1, [r0, #-4]!;    /* push pc to restore at sp to restore; at worst overwrite saved pc */
  ldr   r1, [r13], #4;     /* pop the r0 to restore ... */
  str   r1, [r0, #-4]!;    /* ... and push at sp to restore; at worst overwrite saved lr */
  ldmia r13, {r1-r12};     /* after which we have r0, sp, pc to restore */
  mov   r13, r0;           /* now on the final stack with 2 values to pop */
  popeq {r0,r15};          /* ARM state return */
  mov   r0, r15;           /* switch to Thumb state at here + 0xc */
  add   r0, r0, #5;
  bx    r0;                /* to Thumb at the next address */
  MAYBE_CODE16;
  pop   {r0,r15};
  nop;
#else
  ldr   r14, [r0, #13*4];   /* the sp to restore */
  add   r1, r0, #14*4;
  ldmia r1, {r2,r3};        /* the lr and pc to restore */
  stmdb r14!, {r2,r3};      /* push at sp to restore; at worst overwrite saved lr and pc */
  ldmia r0, {r0-r12};       /* after which we have sp, lr, pc to restore */
  mov   r13, r14;           /* now on the final stack with 2 values to pop */
  pop   {r14};              /* in two steps because pop {r14,r15} is deprecated */
  pop   {r15};
#endif
  MAYBE_CODE16;
#endif
}

/* ----- Development support ----- */

#ifdef VRS_DIAGNOSTICS
static void debug_print_vrs_vfp(uint32_t base, uint64_t *lp)
{
  int c = 0;
  int i;
  for (i = 0; i < 32; i++) {
    printf("D%-2d  0x%16.16llx    ", i + base, *lp);
    lp++;
    if (c++ == 1) {
      c = 0;
      printf("\n");
    }
  }
}


static void debug_print_vrs(_Unwind_Context *context)
{
  phase1_virtual_register_set *vrsp = (phase1_virtual_register_set *)context;
  int i;
  int c;
  printf("------------------------------------------------------------------------\n");
  c = 0;
  for (i = 0; i < 16; i++) {
    printf("r%-2d  0x%8.8x    ", i, vrsp->core.r[i]);
    if (c++ == 3) {
      c = 0;
      printf("\n");
    }
  }

  printf("-----\n");
  if (vrsp->demand_save_vfp_low == 1)
    printf("VFP low registers not saved\n");
  else
    debug_print_vrs_vfp(0, &vrsp->vfp.d[0]);
  printf("-----\n");
  if (vrsp->demand_save_vfp_high == 1)
    printf("VFP high registers not saved\n");
  else
    debug_print_vrs_vfp(16, &vrsp->vfp.d[16]);
  printf("------------------------------------------------------------------------\n");
}
#endif


/* ----- Public routines ----- */

_Unwind_VRS_Result _Unwind_VRS_Set(_Unwind_Context *context,
                                   _Unwind_VRS_RegClass regclass,
                                   uint32_t regno,
                                   _Unwind_VRS_DataRepresentation representation,
                                   void *valuep)
{
  phase1_virtual_register_set *vrsp = (phase1_virtual_register_set *)context;
  switch (regclass) {
  case _UVRSC_CORE:
    {
      if (representation != _UVRSD_UINT32 || regno > 15)
        return _UVRSR_FAILED;
       vrsp->core.r[regno] = *(uint32_t *)valuep;
       return _UVRSR_OK;
    }
  case _UVRSC_VFP:
  case _UVRSC_WMMXD:
  case _UVRSC_WMMXC:
    return _UVRSR_NOT_IMPLEMENTED;
  default:
    break;
  }
  return _UVRSR_FAILED;
}


_Unwind_VRS_Result _Unwind_VRS_Get(_Unwind_Context *context,
                                   _Unwind_VRS_RegClass regclass,
                                   uint32_t regno,
                                   _Unwind_VRS_DataRepresentation representation,
                                   void *valuep)
{
  phase1_virtual_register_set *vrsp = (phase1_virtual_register_set *)context;
  switch (regclass) {
  case _UVRSC_CORE:
    {
      if (representation != _UVRSD_UINT32 || regno > 15)
        return _UVRSR_FAILED;
      *(uint32_t *)valuep = vrsp->core.r[regno];
      return _UVRSR_OK;
    }
  case _UVRSC_VFP:
  case _UVRSC_WMMXD:
  case _UVRSC_WMMXC:
    return _UVRSR_NOT_IMPLEMENTED;
  default:
    break;
  }
  return _UVRSR_FAILED;
}


#define R_SP 13

_Unwind_VRS_Result _Unwind_VRS_Pop(_Unwind_Context *context,
                                   _Unwind_VRS_RegClass regclass,
                                   uint32_t descriminator,
                                   _Unwind_VRS_DataRepresentation representation)
{
  phase1_virtual_register_set *vrsp = (phase1_virtual_register_set *)context;
  switch (regclass) {
  case _UVRSC_CORE:
    {
      /* If SP is included in the mask, the loaded value is used in preference to
       * the writeback value, but only on completion of the loading.
       */
      uint32_t mask, *vsp, *rp, sp_loaded;
      if (representation != _UVRSD_UINT32)
        return _UVRSR_FAILED;
      vsp = (uint32_t *)vrsp->core.r[R_SP];
      rp = (uint32_t *)&vrsp->core;
      mask = descriminator & 0xffff;
      sp_loaded = mask & (1 << R_SP);
      while (mask != 0) {
        if (mask & 1) {
#ifdef VRS_DIAGNOSTICS
          printf("VRS Pop r%d\n", rp - &vrsp->core.r[0]);
#endif
          *rp = *vsp++;
        }
        rp++;
        mask >>= 1;
      }
      if (!sp_loaded)
        vrsp->core.r[R_SP] = (uint32_t)vsp;
      return _UVRSR_OK;
    }
  case _UVRSC_VFP:
#if SUPPORT_VFP
    {
      uint32_t start = descriminator >> 16;
      uint32_t count = descriminator & 0xffff;
      bool some_low = start < 16;
      bool some_high = start + count > 16;
      if ((representation != _UVRSD_VFPX && representation != _UVRSD_DOUBLE) ||
          (representation == _UVRSD_VFPX && some_high) ||
          (representation == _UVRSD_DOUBLE && start + count > 32))
        return _UVRSR_FAILED;
      if (some_low && vrsp->demand_save_vfp_low == 1) { /* Demand-save over phase 1 */
        vrsp->demand_save_vfp_low = 0;
        __ARM_Unwind_VRS_VFPpreserve_low(&vrsp->vfp.d[0]);
      }
      if (some_high && vrsp->demand_save_vfp_high == 1) { /* Demand-save over phase 1 */
        vrsp->demand_save_vfp_high = 0;
        __ARM_Unwind_VRS_VFPpreserve_high(&vrsp->vfp.d[16]);
      }
      /* Now recover from the stack into the real machine registers.
       * Note for _UVRSD_VFPX we assume FSTMX standard format 1.
       * Do this by saving the current VFP registers to a memory area,
       * moving the in-memory values into that area, and
       * restoring from the whole area.
       * Must be careful as the 64-bit values saved by FSTMX might be
       * only 32-bit aligned.
       */
      {
        struct unaligned_vfp_reg_s { uint32_t w1; uint32_t w2; };
        struct unaligned_vfp_reg_s *vsp;
        struct vfp_s temp_vfp;
        if (some_low)
          __ARM_Unwind_VRS_VFPpreserve_low(&temp_vfp.d[0]);
        if (some_high)
          __ARM_Unwind_VRS_VFPpreserve_high(&temp_vfp.d[16]);
        vsp = (struct unaligned_vfp_reg_s *)vrsp->core.r[R_SP];
        while (count--) {
          struct unaligned_vfp_reg_s *v =
            (struct unaligned_vfp_reg_s *)&temp_vfp.d[start++];
          *v = *vsp++;
#ifdef VRS_DIAGNOSTICS
          printf("VRS Pop D%d = 0x%llx\n", start - 1, temp_vfp.d[start - 1]);
#endif
        }
        vrsp->core.r[R_SP] = (uint32_t)((uint32_t *)vsp +
                                        (representation == _UVRSD_VFPX ?
                                         1 : /* +1 to skip the format word */
                                         0));
        if (some_low)
          __ARM_Unwind_VRS_VFPrestore_low(&temp_vfp.d[0]);
        if (some_high)
          __ARM_Unwind_VRS_VFPrestore_high(&temp_vfp.d[16]);
      }
      return _UVRSR_OK;
    }
#endif /* SUPPORT_VFP */
  case _UVRSC_WMMXD:
  case _UVRSC_WMMXC:
    return _UVRSR_NOT_IMPLEMENTED;
  default:
    break;
  }
  return _UVRSR_FAILED;
}



/* =========================              ========================= */
/* ========================= The unwinder ========================= */
/* =========================              ========================= */

/* Index table entry: */

typedef struct __EIT_entry {
  uint32_t fnoffset; /* Place-relative */
  uint32_t content;
} __EIT_entry;


/* Private defines etc: */

static const uint32_t EXIDX_CANTUNWIND = 1;
static const uint32_t uint32_highbit = 0x80000000;

/* ARM C++ personality routines: */

typedef _Unwind_Reason_Code (*personality_routine)(_Unwind_State,
                                                   _Unwind_Control_Block *,
                                                   _Unwind_Context *);

WEAKDECL _Unwind_Reason_Code __aeabi_unwind_cpp_pr0(_Unwind_State state, _Unwind_Control_Block *,
                                                    _Unwind_Context *context);
WEAKDECL _Unwind_Reason_Code __aeabi_unwind_cpp_pr1(_Unwind_State state, _Unwind_Control_Block *,
                                                    _Unwind_Context *context);
WEAKDECL _Unwind_Reason_Code __aeabi_unwind_cpp_pr2(_Unwind_State state, _Unwind_Control_Block *,
                                                    _Unwind_Context *context);


/* Various image symbols: */

struct ExceptionTableInfo {
  uint32_t EIT_base;
  uint32_t EIT_limit;
};
/* We define __ARM_ETInfo to allow access to some linker-generated
   names that are not legal C identifiers. __ARM_ETInfo is extern only
   because of scope limitations of the embedded assembler */
extern const struct ExceptionTableInfo __ARM_ETInfo;
#define EIT_base \
    ((const __EIT_entry *)(__ARM_ETInfo.EIT_base + (const char *)&__ARM_ETInfo))
#define EIT_limit \
    ((const __EIT_entry *)(__ARM_ETInfo.EIT_limit + (const char *)&__ARM_ETInfo))


/* ----- Index table processing ----- */

/* find_and_expand_eit_entry is a support function used in both phases to set
 * ucb.pr_cache and internal cache.
 * Call with a pointer to the ucb and the return address to look up.
 *
 * The table is contained in the half-open interval
 * [EIT_base, EIT_limit) and is an ordered array of __EIT_entrys.
 * Perform a binary search via C library routine bsearch.
 * The table contains only function start addresses (encoded as offsets), so
 * we need to special-case the end table entry in the comparison function,
 * which we do by assuming the function it describes extends to end of memory.
 * This causes us problems indirectly in that we would like to fault as
 * many attempts as possible to look up an invalid return address. There are
 * several ways an invalid return address can be obtained from a broken
 * program, such as someone corrupting the stack or broken unwind instructions
 * recovered the wrong value. It is plausible that many bad return addresses
 * will be either small integers or will point into the heap or stack, hence
 * it's desirable to get the length of that final function roughly right.
 * Here we make no attempt to do it. Code exclusively for use in toolchains
 * which define a suitable limit symbol could make use of that symbol.
 * Alternatively (QoI) a smart linker could augment the index table with a
 * dummy EXIDX_CANTUNWIND entry pointing just past the last real function.
 */

static int EIT_comparator(const void *ck, const void *ce)
{
  uint32_t return_address = *(const uint32_t *)ck;
  const __EIT_entry *eitp = (const __EIT_entry *)ce;
  const __EIT_entry *next_eitp = eitp + 1;
  uint32_t next_fn;
  if (next_eitp != EIT_limit)
    next_fn = __ARM_resolve_prel31((void *)&next_eitp->fnoffset);
  else
    next_fn = 0xffffffffU;
  if (return_address < __ARM_resolve_prel31((void *)&eitp->fnoffset)) return -1;
  if (return_address >= next_fn) return 1;
  return 0;
}


static _Unwind_Reason_Code find_and_expand_eit_entry(_Unwind_Control_Block *ucbp,
                                                     uint32_t return_address)
{
  /* Search the index table for an entry containing the specified return
   * address. Subtract the 2 from the return address, as the index table
   * contains function start addresses (a trailing noreturn BL would
   * appear to return to the first address of the next function (perhaps
   * +1 if Thumb); a leading BL would appear to return to function start
   * + instruction size (perhaps +1 if Thumb)).
   */

  const __EIT_entry *base = EIT_base;
  size_t nelems = EIT_limit - EIT_base;
  __EIT_entry *eitp;

  return_address -= 2;

  eitp = (__EIT_entry *) bsearch(&return_address, base, nelems,
                                 sizeof(__EIT_entry), EIT_comparator);

  if (eitp == NULL) {
    /* The return address we have was not found in the EIT.
     * This breaks the scan and we have to indicate failure.
     */
    ucbp->PR_ADDR = NULL;
    DEBUGGER_BOTTLENECK(ucbp, _UASUBSYS_UNWINDER, _UAACT_ENDING, _UAARG_ENDING_UNWINDER_LOOKUPFAILED);
    return _URC_FAILURE;
  }

  /* Cache the function offset */

  ucbp->pr_cache.fnstart = __ARM_resolve_prel31((void *)&eitp->fnoffset);

  /* Can this frame be unwound at all? */

  if (eitp->content == EXIDX_CANTUNWIND) {
    ucbp->PR_ADDR = NULL;
    DEBUGGER_BOTTLENECK(ucbp, _UASUBSYS_UNWINDER, _UAACT_ENDING, _UAARG_ENDING_NOUNWIND);
    return _URC_FAILURE;
  }

  /* Obtain the address of the "real" __EHT_Header word */

  if (eitp->content & uint32_highbit) {
    /* It is immediate data */
    ucbp->pr_cache.ehtp = (_Unwind_EHT_Header *)&eitp->content;
    ucbp->pr_cache.additional = 1;
  } else {
    /* The content field is a 31-bit place-relative offset to an _Unwind_EHT_Entry structure */
    ucbp->pr_cache.ehtp = (_Unwind_EHT_Header *)__ARM_resolve_prel31((void *)&eitp->content);
    ucbp->pr_cache.additional = 0;
  }

  /* Discover the personality routine address */

  if (*(uint32_t *)(ucbp->pr_cache.ehtp) & uint32_highbit) {
    /* It is immediate data - compute matching pr */
    uint32_t idx = ((*(uint32_t *)(ucbp->pr_cache.ehtp)) >> 24) & 0xf;
    if (idx == 0) ucbp->PR_ADDR = (uint32_t)&__aeabi_unwind_cpp_pr0;
    else if (idx == 1) ucbp->PR_ADDR = (uint32_t)&__aeabi_unwind_cpp_pr1;
    else if (idx == 2) ucbp->PR_ADDR = (uint32_t)&__aeabi_unwind_cpp_pr2;
    else { /* Failed */
      ucbp->PR_ADDR = NULL;
      DEBUGGER_BOTTLENECK(ucbp, _UASUBSYS_UNWINDER, _UAACT_ENDING, _UAARG_ENDING_TABLECORRUPT);
      return _URC_FAILURE;
    }
  } else {
    /* It's a place-relative offset to pr */
    ucbp->PR_ADDR = __ARM_resolve_prel31((void *)(ucbp->pr_cache.ehtp));
  }
  return _URC_OK;
}




/* ----- Unwinding: ----- */

/* Helper fn: If the demand_save flag in a phase1_virtual_register_set was
 * zeroed, the registers were demand-saved. This function restores from
 * the save area.
*/
static FORCEINLINE void restore_non_core_regs(phase1_virtual_register_set *vrsp)
{
#if SUPPORT_VFP
  if (vrsp->demand_save_vfp_low == 0)
    __ARM_Unwind_VRS_VFPrestore_low(&vrsp->vfp.d[0]);
  if (vrsp->demand_save_vfp_high == 0)
    __ARM_Unwind_VRS_VFPrestore_high(&vrsp->vfp.d[16]);
#endif /* SUPPORT_VFP */
}

/* _Unwind_RaiseException is the external entry point to begin unwinding */
__asm _Unwind_Reason_Code _Unwind_RaiseException(_Unwind_Control_Block *ucbp)
{
  extern __ARM_Unwind_RaiseException;

  /* This is a wrapper which creates a phase2_virtual_register_set
   * on the stack, then calls the wrapped function with
   * r0 = ucbp, r1 = phase2_virtual_register_set.
   * If the wrapped function returns, pop the stack and return
   * preserving r0.
   *
   * The phase2_virtual_register_set is created by saving the core
   * registers, carefully writing the original sp value. Note we
   * account for the pc but do not actually write its value here.
   */

  preserve8;

#if HAS_THUMB1_ONLY
  sub      sp,sp,#8*4;              /* make space for r8-r15 */
  push     {r0-r7};                 /* store r0-r7 */
  mov      r1,r14;
  str      r1,[sp,#14*4];           /* store r14 */
  add      r1,sp, #16*4;
  str      r1,[sp,#13*4];           /* store (adjusted value of) r13/sp */
  mov      r1,r12;
  str      r1,[sp,#12*4];           /* store r12 */
  mov      r1,r11;
  str      r1,[sp,#11*4];           /* store r11 */
  mov      r1,r10;
  str      r1,[sp,#10*4];           /* store r10 */
  mov      r1,r9;
  str      r1,[sp,#9*4];            /* store r9 */
  mov      r1,r8;
  str      r1,[sp,#8*4];            /* store r8 */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  movs     r1,#0;
  push     {r1};           /* pushed 1 word => 17 words */
  mov      r1,sp;
  sub      sp,sp,#4;       /* preserve 8 byte alignment => 18 words */
  /* Call the wrapped function. the linker will fix 'bl' => 'blx' as needed */
  bl       __ARM_Unwind_RaiseException;
  ldr      r2,[sp, #14*4+8];
  mov      r14,r2;
  add      sp,sp,#16*4+8;
  bx lr;

#elif HAS_THUMB2 && defined(__thumb)
  str      r14,[sp,#-8]!;
  add      r14,r13,#8;
  str      r14,[sp,#-4]!   /* pushed 3 words => 3 words */
  push     {r0-r12};       /* pushed 13 words => 16 words */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  movs     r1,#0;
  push     {r1};           /* pushed 1 word => 17 words */
  mov      r1,sp;
  sub      sp,sp,#4;       /* preserve 8 byte alignment => 18 words */
  /* Call the wrapped function. the linker will fix 'bl' => 'blx' as needed */
  bl       __ARM_Unwind_RaiseException;
  ldr      r14,[sp,#14*4+8];
  add      sp,sp,#16*4+8;
  bx lr;

#else /* In ARM state, or Thumb1 with ARM available */
  MAYBE_SWITCH_TO_ARM_STATE;

  /* Create a phase2_virtual_register_set on the stack */
  /* Save the core registers, carefully writing the original sp value */
  str r14,[sp,#-8]!;    /* previously 'stmfd sp!,{r14-r15}' - using r15 deprecated */
  add r14, sp, #8;      /* reconstruct original sp */
  str r14, [sp, #-4]!   /* and push it (we've now pushed 3 words) */
  stmfd sp!,{r0-r12};   /* pushed 13 words => 16 words */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  mov r1,#0;
  str r1,[sp,#-4]!;     /* pushed 1 word => 17 words */
  mov r1,sp;
  sub sp,sp,#4;         /* preserve 8 byte alignment => 18 words */
#if OLD_STYLE_INTERWORKING
  ldr r2,Unwind_RaiseException_Offset;
  add r2,r2,pc;
  mov lr,pc;
Offset_Base
  bx    r2;
#else
  /* Call the wrapped function. the linker will fix 'bl' => 'blx' as needed */
  bl  __ARM_Unwind_RaiseException;
#endif
  ldr r14,[sp,#16*4];
  add sp,sp,#18*4;
  RET_LR;
#if OLD_STYLE_INTERWORKING
Unwind_RaiseException_Offset dcd __ARM_Unwind_RaiseException - Offset_Base;
#endif
  MAYBE_CODE16;

#endif

  /* Alternate symbol names for difficult symbols.
   * It is possible no functions included in the image require
   * a handler table. Therefore make only a weak reference to
   * the handler table base symbol, which may be absent.
   */
  align 4
  extern |SHT$$ARM_EXIDX$$Base|;
  extern |SHT$$ARM_EXIDX$$Limit|;
  export __ARM_ETInfo;
  /* these are offsets for /ropi */
__ARM_ETInfo /* layout must match struct ExceptionTableInfo */
eit_base   dcd |SHT$$ARM_EXIDX$$Base|  - __ARM_ETInfo; /* index table base */
eit_limit  dcd |SHT$$ARM_EXIDX$$Limit| - __ARM_ETInfo; /* index table limit */

}


/* __ARM_Unwind_RaiseException performs phase 1 unwinding */

_Unwind_Reason_Code __ARM_Unwind_RaiseException(_Unwind_Control_Block *ucbp,
                                                phase2_virtual_register_set *entry_VRSp)
{
  phase1_virtual_register_set phase1_VRS;

  /* Is this a nested simultaneous propagation?
   * (see comments with _Unwind_Complete)
   */
  if (ucbp->NESTED_CONTEXT == 0) {
    /* No - this is only propagation */
    ucbp->NESTED_CONTEXT = 1;
  } else {
    /* Yes - cache the state elsewhere and restore it when the propagation ends */
    /* This representation wastes space and uses malloc; do better?
     * On the other hand will it ever be used in practice?
     */
    _Unwind_Control_Block *saved_ucbp =
      (_Unwind_Control_Block *)malloc(sizeof(_Unwind_Control_Block));
    if (saved_ucbp == NULL) {
      DEBUGGER_BOTTLENECK(ucbp, _UASUBSYS_UNWINDER, _UAACT_ENDING, _UAARG_ENDING_UNWINDER_BUFFERFAILED);
      return _URC_FAILURE;
    }
    saved_ucbp->unwinder_cache = ucbp->unwinder_cache;
    saved_ucbp->barrier_cache = ucbp->barrier_cache;
    saved_ucbp->cleanup_cache = ucbp->cleanup_cache;
    ucbp->NESTED_CONTEXT = (uint32_t)saved_ucbp;
  }

  /* entry_VRSp contains the core registers as they were when
   * _Unwind_RaiseException was called.  Copy the call-site address to r15
   * then copy all the registers to phase1_VRS for the phase 1 stack scan.
   */

  entry_VRSp->core.r[15] = entry_VRSp->core.r[14];
  phase1_VRS.core = entry_VRSp->core;

  /* For phase 1 only ensure non-core registers are saved before use.
   * If WMMX registers are supported, initialise their flags here and
   * take appropriate action elsewhere.
   */

  phase1_VRS.demand_save_vfp_low = 1;
  phase1_VRS.demand_save_vfp_high = 1;

  /* Now perform a virtual unwind until a propagation barrier is met, or
   * until something goes wrong.  If something does go wrong, we ought (I
   * suppose) to restore registers we may have destroyed.
   */

  while (1) {

    _Unwind_Reason_Code pr_result;

    /* Search the index table for the required entry.  Cache the index table
     * pointer, and obtain and cache the addresses of the "real" __EHT_Header
     * word and the personality routine.
     */

    if (find_and_expand_eit_entry(ucbp, phase1_VRS.core.r[15]) != _URC_OK) {
      restore_non_core_regs(&phase1_VRS);
      /* Debugger bottleneck fn called during lookup */
      return _URC_FAILURE;
    }

    /* Call the pr to decide what to do */

    pr_result = ((personality_routine)ucbp->PR_ADDR)(_US_VIRTUAL_UNWIND_FRAME,
                                                     ucbp,
                                                     (_Unwind_Context *)&phase1_VRS);

    if (pr_result == _URC_HANDLER_FOUND) break;
    if (pr_result == _URC_CONTINUE_UNWIND) continue;

    /* If we get here some sort of failure has occurred in the
     * pr and probably the pr returned _URC_FAILURE
     */
    restore_non_core_regs(&phase1_VRS);
    return _URC_FAILURE;
  }

  /* Propagation barrier located... restore entry register state of non-core regs */

  restore_non_core_regs(&phase1_VRS);

  /* Initiate real unwinding */
  __ARM_unwind_next_frame(ucbp, entry_VRSp);
  /* Unreached, but keep compiler quiet: */
  return _URC_FAILURE;
}


/* __ARM_unwind_next_frame performs phase 2 unwinding */

NORETURNDECL void __ARM_unwind_next_frame(_Unwind_Control_Block *ucbp,
                                          phase2_virtual_register_set *vrsp)
{
  while (1) {

    _Unwind_Reason_Code pr_result;

    /* Search the index table for the required entry.  Cache the index table
     * pointer, and obtain and cache the addresses of the "real" __EHT_Header
     * word and the personality routine.
     */

    if (find_and_expand_eit_entry(ucbp, vrsp->core.r[15]) != _URC_OK)
      abort();

    /* Save the call-site address */

    ucbp->SAVED_CALLSITE_ADDR = vrsp->core.r[15];

    /* Call the pr to do whatever it wants to do on this new frame */

    pr_result = ((personality_routine)ucbp->PR_ADDR)(_US_UNWIND_FRAME_STARTING,
                                                     ucbp,
                                                     (_Unwind_Context *)vrsp);

    if (pr_result == _URC_INSTALL_CONTEXT) {
      /* Upload the registers */
      __ARM_Unwind_VRS_corerestore(&vrsp->core);
    } else if (pr_result == _URC_CONTINUE_UNWIND)
      continue;
    else
      abort();
  }
}


/* _Unwind_Resume is the external entry point called after a cleanup
 * to resume unwinding. It tail-calls a helper function,
 * __ARM_Unwind_Resume, which never returns.
 */
__asm NORETURNDECL void _Unwind_Resume(_Unwind_Control_Block *ucbp)
{
  extern __ARM_Unwind_Resume;

  /* This is a wrapper which creates a phase2_virtual_register_set
   * on the stack, then calls the wrapped function with
   * r0 = ucbp, r1 = phase2_virtual_register_set.
   * If the wrapped function returns, pop the stack and return
   * preserving r0.
   *
   * The phase2_virtual_register_set is created by saving the core
   * registers, carefully writing the original sp value. Note we
   * account for the pc but do not actually write its value here.
   */

  preserve8;

#if HAS_THUMB1_ONLY
  sub      sp,sp,#8*4;              /* make space for r8-r15 */
  push     {r0-r7};                 /* store r0-r7 */
  mov      r1,r14;
  str      r1,[sp,#14*4];           /* store r14 */
  add      r1,sp, #16*4;
  str      r1,[sp,#13*4];           /* store (adjusted value of) r13/sp */
  mov      r1,r12;
  str      r1,[sp,#12*4];           /* store r12 */
  mov      r1,r11;
  str      r1,[sp,#11*4];           /* store r11 */
  mov      r1,r10;
  str      r1,[sp,#10*4];           /* store r10 */
  mov      r1,r9;
  str      r1,[sp,#9*4];            /* store r9 */
  mov      r1,r8;
  str      r1,[sp,#8*4];            /* store r8 */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  movs     r1,#0;
  push     {r1};           /* pushed 1 word => 17 words */
  mov      r1,sp;
  sub      sp,sp,#4;       /* preserve 8 byte alignment => 18 words */
  /* This call never returns */
  ldr r2, Unwind_Resume_Offset;
  add r2,r2,pc;
  bx r2;
Unwind_Resume_Offset_Base
  align
Unwind_Resume_Offset dcd __ARM_Unwind_Resume - Unwind_Resume_Offset_Base;

#elif HAS_THUMB2 && defined(__thumb)
  str      r14,[sp,#-8]!;
  add      r14,r13,#8;
  str      r14,[sp,#-4]!   /* pushed 3 words => 3 words */
  push     {r0-r12};       /* pushed 13 words => 16 words */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  movs     r1,#0;
  push     {r1};           /* pushed 1 word => 17 words */
  mov      r1,sp;
  sub      sp,sp,#4;       /* preserve 8 byte alignment => 18 words */
  /* This call never returns */
  B.W  __ARM_Unwind_Resume;

#else /* In ARM state, or Thumb1 with ARM available */
  MAYBE_SWITCH_TO_ARM_STATE;
  str r14,[sp,#-8]!;    /* previously 'stmfd sp!,{r14-r15}' - using r15 deprecated */
  add r14, sp, #8;      /* reconstruct original sp */
  str r14, [sp, #-4]!   /* and push it (we've now pushed 3 words) */
  stmfd sp!,{r0-r12};   /* pushed 13 words => 16 words */
  /* Write zeroes for the demand_save bytes so no saving occurs in phase 2 */
  mov r1,#0;
  str r1,[sp,#-4]!;     /* pushed 1 word => 17 words */
  mov r1,sp;
  sub sp,sp,#4;         /* preserve 8 byte alignment => 18 words */
  /* This call never returns */
#ifdef __APCS_INTERWORK
  ldr r2,Unwind_Resume_Offset;
  add r2,r2,pc;
  bx    r2;
Unwind_Resume_Offset dcd __ARM_Unwind_Resume - .;
#else
  b __ARM_Unwind_Resume;
#endif
  MAYBE_CODE16;

#endif
}


/* Helper function for _Unwind_Resume */

NORETURNDECL void __ARM_Unwind_Resume(_Unwind_Control_Block *ucbp,
                                      phase2_virtual_register_set *entry_VRSp)
{
  _Unwind_Reason_Code pr_result;

  /* Recover saved state */

  entry_VRSp->core.r[15] = ucbp->SAVED_CALLSITE_ADDR;

  /* Call the cached PR and dispatch */

  pr_result = ((personality_routine)ucbp->PR_ADDR)(_US_UNWIND_FRAME_RESUME,
                                                   ucbp,
                                                   (_Unwind_Context *)entry_VRSp);

  if (pr_result == _URC_INSTALL_CONTEXT) {
   /* Upload the registers */
    __ARM_Unwind_VRS_corerestore(&entry_VRSp->core);
  } else if (pr_result == _URC_CONTINUE_UNWIND)
    __ARM_unwind_next_frame(ucbp, entry_VRSp);
  else
    abort();
}


/* _Unwind_Complete is called at the end of a propagation.
 * If we support multiple simultaneous propagations, restore the cached state
 * of the previous propagation here.
 */

void _Unwind_Complete(_Unwind_Control_Block *ucbp)
{
  if (ucbp->NESTED_CONTEXT == 0) abort();  /* should be impossible */
  if (ucbp->NESTED_CONTEXT == 1) {
    /* This was the only ongoing propagation of this object */
    ucbp->NESTED_CONTEXT--;
    return;
  }
  /* Otherwise we copy the state back from the cache structure pointed to
   * by ucbp->NESTED_CONTEXT.
   */
  {
    _Unwind_Control_Block *context = (_Unwind_Control_Block *)ucbp->NESTED_CONTEXT;
    /* This first one updates ucbp->NESTED_CONTEXT */
    ucbp->unwinder_cache = context->unwinder_cache;
    ucbp->barrier_cache = context->barrier_cache;
    ucbp->cleanup_cache = context->cleanup_cache;
    free(context);
  }
}

#endif /* unwinder_c */
#ifdef unwind_delete_c

/* _Unwind_DeleteException can be used to invoke the exception_cleanup
 * function after catching a foreign exception.
 * The exception cleanup function may start a new exception propagation,
 * so this function needs an unwind table and hence must be built with
 * exceptions enabled.
 */

#pragma exceptions_unwind

void _Unwind_DeleteException(_Unwind_Control_Block *ucbp)
{
  if (ucbp->exception_cleanup != NULL)
    (ucbp->exception_cleanup)(_URC_FOREIGN_EXCEPTION_CAUGHT, ucbp);
}

#endif /* unwind_delete_c */
#ifdef unwind_activity_c

/* Runtime debug "bottleneck function": */
/* (not in the current Exceptions EABI document) */

#ifdef UNWIND_ACTIVITY_DIAGNOSTICS
void _Unwind_Activity(_Unwind_Control_Block *ucbp, uint32_t reason, uint32_t arg)
{
  uint32_t who = reason >> 24;
  uint32_t activity = reason & 0xffffff;
  printf("_Unwind_Activity: UCB=0x%8.8x Reason=(", (uint32_t)ucbp);
  switch (who) {
  case _UASUBSYS_UNWINDER:
    printf("unw,");
    if (activity >= 0x80)
      printf("%x) Arg=0x%8.8x\n", activity, arg);
    break;
  case _UASUBSYS_CPP:
    printf("C++,");
    if (activity >= 0x80) {
      if (activity == _UAACT_CPP_TYPEINFO)
        printf("typeinfo) Typeinfo=0x%8.8x\n", arg);
      else
        printf("%x) Arg=0x%8.8x\n", activity, arg);
    }
    break;
  default:
    printf("???,");
    if (activity >= 0x80)
      printf("%x) Arg=0x%8.8x\n", activity, arg);
    break;
  }
  if (activity < 0x80) {
    switch (activity) {
    case _UAACT_STARTING:
      printf("starting) Typeinfo=0x%8.8x\n", arg);
      break;
    case _UAACT_ENDING:
      printf("ending) Cause=%d\n", arg);
      break;
    case _UAACT_BARRIERFOUND:
      printf("barrierfound) Pad=0x%8.8x\n", arg);
      break;
    case _UAACT_PADENTRY:
      printf("padentry) Pad=0x%8.8x\n", arg);
      break;
    default:
      printf("%x) Arg=0x%8.8x\n", activity, arg);
      break;
    }
  }
}
#else
/* This definition is intended to defeat simple armlink function inlining.
 */
__asm void _Unwind_Activity(_Unwind_Control_Block *ucbp, uint32_t reason, uint32_t arg)
{
    RET_LR;
    NOP;    /* does not end with a return */
}
#endif

#endif /* unwind_activity_c */
