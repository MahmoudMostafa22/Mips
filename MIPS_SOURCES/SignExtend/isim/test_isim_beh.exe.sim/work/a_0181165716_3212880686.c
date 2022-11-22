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
static const char *ng0 = "D:/Documents/Year 3/Term 6/Xilinx Programs/Mips/Mahmoud_Mostafa_ALU/SignExtend/SignExtend32Bit.vhd";
extern char *IEEE_P_3620187407;

char *ieee_p_3620187407_sub_767668596_3965413181(char *, char *, char *, char *, char *, char *);


static void work_a_0181165716_3212880686_p_0(char *t0)
{
    char t9[16];
    char t12[16];
    char *t1;
    char *t2;
    int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned char t7;
    unsigned char t8;
    char *t10;
    char *t13;
    char *t14;
    int t15;
    unsigned int t16;
    char *t17;
    char *t18;
    char *t19;
    unsigned int t20;
    unsigned char t21;
    char *t22;
    char *t23;
    char *t24;
    char *t25;
    char *t26;

LAB0:    xsi_set_current_line(44, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t3 = (15 - 15);
    t4 = (t3 * -1);
    t5 = (1U * t4);
    t6 = (0 + t5);
    t1 = (t2 + t6);
    t7 = *((unsigned char *)t1);
    t8 = (t7 == (unsigned char)2);
    if (t8 != 0)
        goto LAB2;

LAB4:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t3 = (15 - 15);
    t4 = (t3 * -1);
    t5 = (1U * t4);
    t6 = (0 + t5);
    t1 = (t2 + t6);
    t7 = *((unsigned char *)t1);
    t8 = (t7 == (unsigned char)3);
    if (t8 != 0)
        goto LAB7;

LAB8:
LAB3:    t1 = (t0 + 2672);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(44, ng0);
    t10 = (t0 + 4612);
    t13 = (t12 + 0U);
    t14 = (t13 + 0U);
    *((int *)t14) = 0;
    t14 = (t13 + 4U);
    *((int *)t14) = 31;
    t14 = (t13 + 8U);
    *((int *)t14) = 1;
    t15 = (31 - 0);
    t16 = (t15 * 1);
    t16 = (t16 + 1);
    t14 = (t13 + 12U);
    *((unsigned int *)t14) = t16;
    t14 = (t0 + 1032U);
    t17 = *((char **)t14);
    t14 = (t0 + 4556U);
    t18 = ieee_p_3620187407_sub_767668596_3965413181(IEEE_P_3620187407, t9, t10, t12, t17, t14);
    t19 = (t9 + 12U);
    t16 = *((unsigned int *)t19);
    t20 = (1U * t16);
    t21 = (32U != t20);
    if (t21 == 1)
        goto LAB5;

LAB6:    t22 = (t0 + 2752);
    t23 = (t22 + 56U);
    t24 = *((char **)t23);
    t25 = (t24 + 56U);
    t26 = *((char **)t25);
    memcpy(t26, t18, 32U);
    xsi_driver_first_trans_fast_port(t22);
    goto LAB3;

LAB5:    xsi_size_not_matching(32U, t20, 0);
    goto LAB6;

LAB7:    xsi_set_current_line(45, ng0);
    t10 = (t0 + 4644);
    t13 = (t12 + 0U);
    t14 = (t13 + 0U);
    *((int *)t14) = 0;
    t14 = (t13 + 4U);
    *((int *)t14) = 31;
    t14 = (t13 + 8U);
    *((int *)t14) = 1;
    t15 = (31 - 0);
    t16 = (t15 * 1);
    t16 = (t16 + 1);
    t14 = (t13 + 12U);
    *((unsigned int *)t14) = t16;
    t14 = (t0 + 1032U);
    t17 = *((char **)t14);
    t14 = (t0 + 4556U);
    t18 = ieee_p_3620187407_sub_767668596_3965413181(IEEE_P_3620187407, t9, t10, t12, t17, t14);
    t19 = (t9 + 12U);
    t16 = *((unsigned int *)t19);
    t20 = (1U * t16);
    t21 = (32U != t20);
    if (t21 == 1)
        goto LAB9;

LAB10:    t22 = (t0 + 2752);
    t23 = (t22 + 56U);
    t24 = *((char **)t23);
    t25 = (t24 + 56U);
    t26 = *((char **)t25);
    memcpy(t26, t18, 32U);
    xsi_driver_first_trans_fast_port(t22);
    goto LAB3;

LAB9:    xsi_size_not_matching(32U, t20, 0);
    goto LAB10;

}


extern void work_a_0181165716_3212880686_init()
{
	static char *pe[] = {(void *)work_a_0181165716_3212880686_p_0};
	xsi_register_didat("work_a_0181165716_3212880686", "isim/test_isim_beh.exe.sim/work/a_0181165716_3212880686.didat");
	xsi_register_executes(pe);
}
