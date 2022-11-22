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
static const char *ng0 = "D:/Documents/Year 3/Term 6/Xilinx Programs/Mips/MahmoudMostafa_19102461_2021/MIPS_SOURCES/MahmoudMostafa_memUnit/MemoryUnit.vhd";
extern char *IEEE_P_1242562249;
extern char *IEEE_P_2592010699;

int ieee_p_1242562249_sub_1657552908_1035706684(char *, char *, char *);
char *ieee_p_1242562249_sub_1919365254_1035706684(char *, char *, char *, char *, int );
unsigned char ieee_p_2592010699_sub_1744673427_503743352(char *, char *, unsigned int , unsigned int );


static void work_a_1682115572_3212880686_p_0(char *t0)
{
    char t22[16];
    unsigned char t1;
    char *t2;
    char *t3;
    unsigned char t4;
    unsigned char t5;
    char *t6;
    unsigned char t7;
    unsigned char t8;
    char *t9;
    char *t10;
    int t11;
    int t12;
    unsigned int t13;
    unsigned int t14;
    unsigned int t15;
    char *t16;
    char *t17;
    char *t18;
    char *t19;
    char *t20;
    char *t21;
    unsigned char t23;
    unsigned char t24;
    unsigned int t25;
    unsigned int t26;
    unsigned int t27;
    char *t28;

LAB0:    xsi_set_current_line(56, ng0);
    t2 = (t0 + 1032U);
    t3 = *((char **)t2);
    t4 = *((unsigned char *)t3);
    t5 = (t4 == (unsigned char)3);
    if (t5 == 1)
        goto LAB5;

LAB6:    t1 = (unsigned char)0;

LAB7:    if (t1 != 0)
        goto LAB2;

LAB4:    t2 = (t0 + 1032U);
    t3 = *((char **)t2);
    t5 = *((unsigned char *)t3);
    t7 = (t5 == (unsigned char)2);
    if (t7 == 1)
        goto LAB13;

LAB14:    t4 = (unsigned char)0;

LAB15:    if (t4 == 1)
        goto LAB10;

LAB11:    t1 = (unsigned char)0;

LAB12:    if (t1 != 0)
        goto LAB8;

LAB9:
LAB3:    t2 = (t0 + 3472);
    *((int *)t2) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(57, ng0);
    t2 = (t0 + 1992U);
    t9 = *((char **)t2);
    t2 = (t0 + 1352U);
    t10 = *((char **)t2);
    t2 = (t0 + 6624U);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t10, t2);
    t12 = (t11 - 0);
    t13 = (t12 * 1);
    xsi_vhdl_check_range_of_index(0, 35, 1, t11);
    t14 = (8U * t13);
    t15 = (0 + t14);
    t16 = (t9 + t15);
    t17 = (t0 + 3552);
    t18 = (t17 + 56U);
    t19 = *((char **)t18);
    t20 = (t19 + 56U);
    t21 = *((char **)t20);
    memcpy(t21, t16, 8U);
    xsi_driver_first_trans_delta(t17, 0U, 8U, 0LL);
    xsi_set_current_line(58, ng0);
    t2 = (t0 + 1992U);
    t3 = *((char **)t2);
    t2 = (t0 + 1352U);
    t6 = *((char **)t2);
    t2 = (t0 + 6624U);
    t9 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t6, t2, 1);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t9, t22);
    t12 = (t11 - 0);
    t13 = (t12 * 1);
    xsi_vhdl_check_range_of_index(0, 35, 1, t11);
    t14 = (8U * t13);
    t15 = (0 + t14);
    t10 = (t3 + t15);
    t16 = (t0 + 3552);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t10, 8U);
    xsi_driver_first_trans_delta(t16, 8U, 8U, 0LL);
    xsi_set_current_line(59, ng0);
    t2 = (t0 + 1992U);
    t3 = *((char **)t2);
    t2 = (t0 + 1352U);
    t6 = *((char **)t2);
    t2 = (t0 + 6624U);
    t9 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t6, t2, 2);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t9, t22);
    t12 = (t11 - 0);
    t13 = (t12 * 1);
    xsi_vhdl_check_range_of_index(0, 35, 1, t11);
    t14 = (8U * t13);
    t15 = (0 + t14);
    t10 = (t3 + t15);
    t16 = (t0 + 3552);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t10, 8U);
    xsi_driver_first_trans_delta(t16, 16U, 8U, 0LL);
    xsi_set_current_line(60, ng0);
    t2 = (t0 + 1992U);
    t3 = *((char **)t2);
    t2 = (t0 + 1352U);
    t6 = *((char **)t2);
    t2 = (t0 + 6624U);
    t9 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t6, t2, 3);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t9, t22);
    t12 = (t11 - 0);
    t13 = (t12 * 1);
    xsi_vhdl_check_range_of_index(0, 35, 1, t11);
    t14 = (8U * t13);
    t15 = (0 + t14);
    t10 = (t3 + t15);
    t16 = (t0 + 3552);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t10, 8U);
    xsi_driver_first_trans_delta(t16, 24U, 8U, 0LL);
    goto LAB3;

LAB5:    t2 = (t0 + 1192U);
    t6 = *((char **)t2);
    t7 = *((unsigned char *)t6);
    t8 = (t7 == (unsigned char)2);
    t1 = t8;
    goto LAB7;

LAB8:    xsi_set_current_line(62, ng0);
    t9 = (t0 + 1512U);
    t10 = *((char **)t9);
    t13 = (31 - 31);
    t14 = (t13 * 1U);
    t15 = (0 + t14);
    t9 = (t10 + t15);
    t16 = (t0 + 1352U);
    t17 = *((char **)t16);
    t16 = (t0 + 6624U);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t17, t16);
    t12 = (t11 - 0);
    t25 = (t12 * 1);
    t26 = (8U * t25);
    t27 = (0U + t26);
    t18 = (t0 + 3616);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    t21 = (t20 + 56U);
    t28 = *((char **)t21);
    memcpy(t28, t9, 8U);
    xsi_driver_first_trans_delta(t18, t27, 8U, 0LL);
    xsi_set_current_line(63, ng0);
    t2 = (t0 + 1512U);
    t3 = *((char **)t2);
    t13 = (31 - 23);
    t14 = (t13 * 1U);
    t15 = (0 + t14);
    t2 = (t3 + t15);
    t6 = (t0 + 1352U);
    t9 = *((char **)t6);
    t6 = (t0 + 6624U);
    t10 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t9, t6, 1);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t10, t22);
    t12 = (t11 - 0);
    t25 = (t12 * 1);
    t26 = (8U * t25);
    t27 = (0U + t26);
    t16 = (t0 + 3616);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t2, 8U);
    xsi_driver_first_trans_delta(t16, t27, 8U, 0LL);
    xsi_set_current_line(64, ng0);
    t2 = (t0 + 1512U);
    t3 = *((char **)t2);
    t13 = (31 - 15);
    t14 = (t13 * 1U);
    t15 = (0 + t14);
    t2 = (t3 + t15);
    t6 = (t0 + 1352U);
    t9 = *((char **)t6);
    t6 = (t0 + 6624U);
    t10 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t9, t6, 2);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t10, t22);
    t12 = (t11 - 0);
    t25 = (t12 * 1);
    t26 = (8U * t25);
    t27 = (0U + t26);
    t16 = (t0 + 3616);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t2, 8U);
    xsi_driver_first_trans_delta(t16, t27, 8U, 0LL);
    xsi_set_current_line(65, ng0);
    t2 = (t0 + 1512U);
    t3 = *((char **)t2);
    t13 = (31 - 7);
    t14 = (t13 * 1U);
    t15 = (0 + t14);
    t2 = (t3 + t15);
    t6 = (t0 + 1352U);
    t9 = *((char **)t6);
    t6 = (t0 + 6624U);
    t10 = ieee_p_1242562249_sub_1919365254_1035706684(IEEE_P_1242562249, t22, t9, t6, 3);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t10, t22);
    t12 = (t11 - 0);
    t25 = (t12 * 1);
    t26 = (8U * t25);
    t27 = (0U + t26);
    t16 = (t0 + 3616);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t2, 8U);
    xsi_driver_first_trans_delta(t16, t27, 8U, 0LL);
    goto LAB3;

LAB10:    t2 = (t0 + 1792U);
    t24 = ieee_p_2592010699_sub_1744673427_503743352(IEEE_P_2592010699, t2, 0U, 0U);
    t1 = t24;
    goto LAB12;

LAB13:    t2 = (t0 + 1192U);
    t6 = *((char **)t2);
    t8 = *((unsigned char *)t6);
    t23 = (t8 == (unsigned char)3);
    t4 = t23;
    goto LAB15;

}


extern void work_a_1682115572_3212880686_init()
{
	static char *pe[] = {(void *)work_a_1682115572_3212880686_p_0};
	xsi_register_didat("work_a_1682115572_3212880686", "isim/TESTMIPS_isim_beh.exe.sim/work/a_1682115572_3212880686.didat");
	xsi_register_executes(pe);
}
