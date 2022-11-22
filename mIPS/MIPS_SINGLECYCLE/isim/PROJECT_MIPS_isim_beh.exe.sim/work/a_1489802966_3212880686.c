/**********************************************************************/
/*   ____  ____                                                       */
/*  /   /\/   /                                                       */
/* /___/  \  /                                                        */
/* \   \   \/                                                       */
/*  \   \        Copyright (c) 2003-2009 Xilinx, Inc.                */
/*  /   /          All Right Reserved.                                 */
/* /---/   /\                                                         */
/* \   \  /  \                                                      */
/*  \___\/\___\                                                    */
/***********************************************************************/

/* This file is designed for use with ISim build 0x7708f090 */

#define XSI_HIDE_SYMBOL_SPEC true
#include "xsi.h"
#include <memory.h>
#ifdef __GNUC__
#include <stdlib.h>
#else
#include <malloc.h>
#define alloca _alloca
#endif
static const char *ng0 = "D:/Documents/Year 3/Term 6/Xilinx Programs/Mips/MahmoudMostafa_19102461_2021/MIPS_SOURCES/ShiftLeft2Bit/ShiftLeft.vhd";



static void work_a_1489802966_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    unsigned char t3;
    char *t4;
    char *t5;
    char *t6;
    char *t7;
    char *t8;
    unsigned int t9;
    unsigned int t10;
    unsigned int t11;

LAB0:    xsi_set_current_line(43, ng0);
    t1 = (t0 + 4692);
    t3 = (2U != 2U);
    if (t3 == 1)
        goto LAB2;

LAB3:    t4 = (t0 + 2752);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    t7 = (t6 + 56U);
    t8 = *((char **)t7);
    memcpy(t8, t1, 2U);
    xsi_driver_first_trans_delta(t4, 30U, 2U, 0LL);
    xsi_set_current_line(44, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t9 = (31 - 29);
    t10 = (t9 * 1U);
    t11 = (0 + t10);
    t1 = (t2 + t11);
    t4 = (t0 + 2752);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    t7 = (t6 + 56U);
    t8 = *((char **)t7);
    memcpy(t8, t1, 30U);
    xsi_driver_first_trans_delta(t4, 0U, 30U, 0LL);
    t1 = (t0 + 2672);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_size_not_matching(2U, 2U, 0);
    goto LAB3;

}


extern void work_a_1489802966_3212880686_init()
{
	static char *pe[] = {(void *)work_a_1489802966_3212880686_p_0};
	xsi_register_didat("work_a_1489802966_3212880686", "isim/PROJECT_MIPS_isim_beh.exe.sim/work/a_1489802966_3212880686.didat");
	xsi_register_executes(pe);
}
