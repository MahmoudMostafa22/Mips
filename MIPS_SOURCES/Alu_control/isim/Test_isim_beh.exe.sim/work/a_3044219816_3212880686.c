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
static const char *ng0 = "D:/Documents/Year 3/Term 6/Xilinx Programs/Week 2/Alu_control/ALU_Control.vhd";



static void work_a_3044219816_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    char *t3;
    unsigned char t4;
    unsigned int t5;
    char *t6;
    char *t7;
    char *t8;
    char *t9;
    char *t10;
    char *t11;
    char *t12;
    char *t13;
    char *t14;
    unsigned char t15;
    unsigned char t16;
    unsigned char t17;
    unsigned char t18;
    int t19;
    unsigned int t20;
    unsigned int t21;
    unsigned int t22;
    unsigned char t23;
    unsigned char t24;
    int t25;
    unsigned int t26;
    unsigned int t27;
    unsigned int t28;
    unsigned char t29;
    unsigned char t30;
    int t31;
    unsigned int t32;
    unsigned int t33;
    unsigned int t34;
    unsigned char t35;
    unsigned char t36;
    char *t37;
    int t38;
    unsigned int t39;
    unsigned int t40;
    unsigned int t41;
    unsigned char t42;
    unsigned char t43;
    char *t44;
    char *t45;
    char *t46;
    char *t47;
    char *t48;
    char *t49;
    char *t50;
    int t51;
    unsigned int t52;
    unsigned int t53;
    unsigned char t54;

LAB0:    xsi_set_current_line(43, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t1 = (t0 + 4752);
    t4 = 1;
    if (2U == 2U)
        goto LAB5;

LAB6:    t4 = 0;

LAB7:    if (t4 != 0)
        goto LAB2;

LAB4:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t1 = (t0 + 4758);
    t4 = 1;
    if (2U == 2U)
        goto LAB13;

LAB14:    t4 = 0;

LAB15:    if (t4 != 0)
        goto LAB11;

LAB12:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t1 = (t0 + 4764);
    t15 = 1;
    if (2U == 2U)
        goto LAB24;

LAB25:    t15 = 0;

LAB26:    if (t15 == 1)
        goto LAB21;

LAB22:    t4 = (unsigned char)0;

LAB23:    if (t4 != 0)
        goto LAB19;

LAB20:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t1 = (t0 + 4770);
    t15 = 1;
    if (2U == 2U)
        goto LAB44;

LAB45:    t15 = 0;

LAB46:    if (t15 == 1)
        goto LAB41;

LAB42:    t4 = (unsigned char)0;

LAB43:    if (t4 != 0)
        goto LAB39;

LAB40:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t1 = (t0 + 4776);
    t15 = 1;
    if (2U == 2U)
        goto LAB64;

LAB65:    t15 = 0;

LAB66:    if (t15 == 1)
        goto LAB61;

LAB62:    t4 = (unsigned char)0;

LAB63:    if (t4 != 0)
        goto LAB59;

LAB60:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t19 = (1 - 1);
    t5 = (t19 * -1);
    t20 = (1U * t5);
    t21 = (0 + t20);
    t1 = (t2 + t21);
    t15 = *((unsigned char *)t1);
    t16 = (t15 == (unsigned char)3);
    if (t16 == 1)
        goto LAB81;

LAB82:    t4 = (unsigned char)0;

LAB83:    if (t4 != 0)
        goto LAB79;

LAB80:    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t19 = (1 - 1);
    t5 = (t19 * -1);
    t20 = (1U * t5);
    t21 = (0 + t20);
    t1 = (t2 + t21);
    t15 = *((unsigned char *)t1);
    t16 = (t15 == (unsigned char)3);
    if (t16 == 1)
        goto LAB95;

LAB96:    t4 = (unsigned char)0;

LAB97:    if (t4 != 0)
        goto LAB93;

LAB94:
LAB3:    t1 = (t0 + 2832);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(43, ng0);
    t8 = (t0 + 4754);
    t10 = (t0 + 2912);
    t11 = (t10 + 56U);
    t12 = *((char **)t11);
    t13 = (t12 + 56U);
    t14 = *((char **)t13);
    memcpy(t14, t8, 4U);
    xsi_driver_first_trans_fast_port(t10);
    goto LAB3;

LAB5:    t5 = 0;

LAB8:    if (t5 < 2U)
        goto LAB9;
    else
        goto LAB7;

LAB9:    t6 = (t2 + t5);
    t7 = (t1 + t5);
    if (*((unsigned char *)t6) != *((unsigned char *)t7))
        goto LAB6;

LAB10:    t5 = (t5 + 1);
    goto LAB8;

LAB11:    xsi_set_current_line(44, ng0);
    t8 = (t0 + 4760);
    t10 = (t0 + 2912);
    t11 = (t10 + 56U);
    t12 = *((char **)t11);
    t13 = (t12 + 56U);
    t14 = *((char **)t13);
    memcpy(t14, t8, 4U);
    xsi_driver_first_trans_fast_port(t10);
    goto LAB3;

LAB13:    t5 = 0;

LAB16:    if (t5 < 2U)
        goto LAB17;
    else
        goto LAB15;

LAB17:    t6 = (t2 + t5);
    t7 = (t1 + t5);
    if (*((unsigned char *)t6) != *((unsigned char *)t7))
        goto LAB14;

LAB18:    t5 = (t5 + 1);
    goto LAB16;

LAB19:    xsi_set_current_line(45, ng0);
    t44 = (t0 + 4766);
    t46 = (t0 + 2912);
    t47 = (t46 + 56U);
    t48 = *((char **)t47);
    t49 = (t48 + 56U);
    t50 = *((char **)t49);
    memcpy(t50, t44, 4U);
    xsi_driver_first_trans_fast_port(t46);
    goto LAB3;

LAB21:    t8 = (t0 + 1192U);
    t9 = *((char **)t8);
    t19 = (3 - 5);
    t20 = (t19 * -1);
    t21 = (1U * t20);
    t22 = (0 + t21);
    t8 = (t9 + t22);
    t23 = *((unsigned char *)t8);
    t24 = (t23 == (unsigned char)2);
    if (t24 == 1)
        goto LAB36;

LAB37:    t18 = (unsigned char)0;

LAB38:    if (t18 == 1)
        goto LAB33;

LAB34:    t17 = (unsigned char)0;

LAB35:    if (t17 == 1)
        goto LAB30;

LAB31:    t16 = (unsigned char)0;

LAB32:    t4 = t16;
    goto LAB23;

LAB24:    t5 = 0;

LAB27:    if (t5 < 2U)
        goto LAB28;
    else
        goto LAB26;

LAB28:    t6 = (t2 + t5);
    t7 = (t1 + t5);
    if (*((unsigned char *)t6) != *((unsigned char *)t7))
        goto LAB25;

LAB29:    t5 = (t5 + 1);
    goto LAB27;

LAB30:    t14 = (t0 + 1192U);
    t37 = *((char **)t14);
    t38 = (0 - 5);
    t39 = (t38 * -1);
    t40 = (1U * t39);
    t41 = (0 + t40);
    t14 = (t37 + t41);
    t42 = *((unsigned char *)t14);
    t43 = (t42 == (unsigned char)2);
    t16 = t43;
    goto LAB32;

LAB33:    t12 = (t0 + 1192U);
    t13 = *((char **)t12);
    t31 = (1 - 5);
    t32 = (t31 * -1);
    t33 = (1U * t32);
    t34 = (0 + t33);
    t12 = (t13 + t34);
    t35 = *((unsigned char *)t12);
    t36 = (t35 == (unsigned char)2);
    t17 = t36;
    goto LAB35;

LAB36:    t10 = (t0 + 1192U);
    t11 = *((char **)t10);
    t25 = (2 - 5);
    t26 = (t25 * -1);
    t27 = (1U * t26);
    t28 = (0 + t27);
    t10 = (t11 + t28);
    t29 = *((unsigned char *)t10);
    t30 = (t29 == (unsigned char)2);
    t18 = t30;
    goto LAB38;

LAB39:    xsi_set_current_line(46, ng0);
    t44 = (t0 + 4772);
    t46 = (t0 + 2912);
    t47 = (t46 + 56U);
    t48 = *((char **)t47);
    t49 = (t48 + 56U);
    t50 = *((char **)t49);
    memcpy(t50, t44, 4U);
    xsi_driver_first_trans_fast_port(t46);
    goto LAB3;

LAB41:    t8 = (t0 + 1192U);
    t9 = *((char **)t8);
    t19 = (3 - 5);
    t20 = (t19 * -1);
    t21 = (1U * t20);
    t22 = (0 + t21);
    t8 = (t9 + t22);
    t23 = *((unsigned char *)t8);
    t24 = (t23 == (unsigned char)2);
    if (t24 == 1)
        goto LAB56;

LAB57:    t18 = (unsigned char)0;

LAB58:    if (t18 == 1)
        goto LAB53;

LAB54:    t17 = (unsigned char)0;

LAB55:    if (t17 == 1)
        goto LAB50;

LAB51:    t16 = (unsigned char)0;

LAB52:    t4 = t16;
    goto LAB43;

LAB44:    t5 = 0;

LAB47:    if (t5 < 2U)
        goto LAB48;
    else
        goto LAB46;

LAB48:    t6 = (t2 + t5);
    t7 = (t1 + t5);
    if (*((unsigned char *)t6) != *((unsigned char *)t7))
        goto LAB45;

LAB49:    t5 = (t5 + 1);
    goto LAB47;

LAB50:    t14 = (t0 + 1192U);
    t37 = *((char **)t14);
    t38 = (0 - 5);
    t39 = (t38 * -1);
    t40 = (1U * t39);
    t41 = (0 + t40);
    t14 = (t37 + t41);
    t42 = *((unsigned char *)t14);
    t43 = (t42 == (unsigned char)2);
    t16 = t43;
    goto LAB52;

LAB53:    t12 = (t0 + 1192U);
    t13 = *((char **)t12);
    t31 = (1 - 5);
    t32 = (t31 * -1);
    t33 = (1U * t32);
    t34 = (0 + t33);
    t12 = (t13 + t34);
    t35 = *((unsigned char *)t12);
    t36 = (t35 == (unsigned char)2);
    t17 = t36;
    goto LAB55;

LAB56:    t10 = (t0 + 1192U);
    t11 = *((char **)t10);
    t25 = (2 - 5);
    t26 = (t25 * -1);
    t27 = (1U * t26);
    t28 = (0 + t27);
    t10 = (t11 + t28);
    t29 = *((unsigned char *)t10);
    t30 = (t29 == (unsigned char)3);
    t18 = t30;
    goto LAB58;

LAB59:    xsi_set_current_line(47, ng0);
    t44 = (t0 + 4778);
    t46 = (t0 + 2912);
    t47 = (t46 + 56U);
    t48 = *((char **)t47);
    t49 = (t48 + 56U);
    t50 = *((char **)t49);
    memcpy(t50, t44, 4U);
    xsi_driver_first_trans_fast_port(t46);
    goto LAB3;

LAB61:    t8 = (t0 + 1192U);
    t9 = *((char **)t8);
    t19 = (3 - 5);
    t20 = (t19 * -1);
    t21 = (1U * t20);
    t22 = (0 + t21);
    t8 = (t9 + t22);
    t23 = *((unsigned char *)t8);
    t24 = (t23 == (unsigned char)2);
    if (t24 == 1)
        goto LAB76;

LAB77:    t18 = (unsigned char)0;

LAB78:    if (t18 == 1)
        goto LAB73;

LAB74:    t17 = (unsigned char)0;

LAB75:    if (t17 == 1)
        goto LAB70;

LAB71:    t16 = (unsigned char)0;

LAB72:    t4 = t16;
    goto LAB63;

LAB64:    t5 = 0;

LAB67:    if (t5 < 2U)
        goto LAB68;
    else
        goto LAB66;

LAB68:    t6 = (t2 + t5);
    t7 = (t1 + t5);
    if (*((unsigned char *)t6) != *((unsigned char *)t7))
        goto LAB65;

LAB69:    t5 = (t5 + 1);
    goto LAB67;

LAB70:    t14 = (t0 + 1192U);
    t37 = *((char **)t14);
    t38 = (0 - 5);
    t39 = (t38 * -1);
    t40 = (1U * t39);
    t41 = (0 + t40);
    t14 = (t37 + t41);
    t42 = *((unsigned char *)t14);
    t43 = (t42 == (unsigned char)3);
    t16 = t43;
    goto LAB72;

LAB73:    t12 = (t0 + 1192U);
    t13 = *((char **)t12);
    t31 = (1 - 5);
    t32 = (t31 * -1);
    t33 = (1U * t32);
    t34 = (0 + t33);
    t12 = (t13 + t34);
    t35 = *((unsigned char *)t12);
    t36 = (t35 == (unsigned char)2);
    t17 = t36;
    goto LAB75;

LAB76:    t10 = (t0 + 1192U);
    t11 = *((char **)t10);
    t25 = (2 - 5);
    t26 = (t25 * -1);
    t27 = (1U * t26);
    t28 = (0 + t27);
    t10 = (t11 + t28);
    t29 = *((unsigned char *)t10);
    t30 = (t29 == (unsigned char)3);
    t18 = t30;
    goto LAB78;

LAB79:    xsi_set_current_line(48, ng0);
    t13 = (t0 + 4782);
    t37 = (t0 + 2912);
    t44 = (t37 + 56U);
    t45 = *((char **)t44);
    t46 = (t45 + 56U);
    t47 = *((char **)t46);
    memcpy(t47, t13, 4U);
    xsi_driver_first_trans_fast_port(t37);
    goto LAB3;

LAB81:    t3 = (t0 + 1192U);
    t6 = *((char **)t3);
    t25 = (3 - 5);
    t22 = (t25 * -1);
    t26 = (1U * t22);
    t27 = (0 + t26);
    t3 = (t6 + t27);
    t24 = *((unsigned char *)t3);
    t29 = (t24 == (unsigned char)2);
    if (t29 == 1)
        goto LAB90;

LAB91:    t23 = (unsigned char)0;

LAB92:    if (t23 == 1)
        goto LAB87;

LAB88:    t18 = (unsigned char)0;

LAB89:    if (t18 == 1)
        goto LAB84;

LAB85:    t17 = (unsigned char)0;

LAB86:    t4 = t17;
    goto LAB83;

LAB84:    t11 = (t0 + 1192U);
    t12 = *((char **)t11);
    t51 = (0 - 5);
    t41 = (t51 * -1);
    t52 = (1U * t41);
    t53 = (0 + t52);
    t11 = (t12 + t53);
    t43 = *((unsigned char *)t11);
    t54 = (t43 == (unsigned char)2);
    t17 = t54;
    goto LAB86;

LAB87:    t9 = (t0 + 1192U);
    t10 = *((char **)t9);
    t38 = (1 - 5);
    t34 = (t38 * -1);
    t39 = (1U * t34);
    t40 = (0 + t39);
    t9 = (t10 + t40);
    t36 = *((unsigned char *)t9);
    t42 = (t36 == (unsigned char)3);
    t18 = t42;
    goto LAB89;

LAB90:    t7 = (t0 + 1192U);
    t8 = *((char **)t7);
    t31 = (2 - 5);
    t28 = (t31 * -1);
    t32 = (1U * t28);
    t33 = (0 + t32);
    t7 = (t8 + t33);
    t30 = *((unsigned char *)t7);
    t35 = (t30 == (unsigned char)2);
    t23 = t35;
    goto LAB92;

LAB93:    xsi_set_current_line(49, ng0);
    t13 = (t0 + 4786);
    t37 = (t0 + 2912);
    t44 = (t37 + 56U);
    t45 = *((char **)t44);
    t46 = (t45 + 56U);
    t47 = *((char **)t46);
    memcpy(t47, t13, 4U);
    xsi_driver_first_trans_fast_port(t37);
    goto LAB3;

LAB95:    t3 = (t0 + 1192U);
    t6 = *((char **)t3);
    t25 = (3 - 5);
    t22 = (t25 * -1);
    t26 = (1U * t22);
    t27 = (0 + t26);
    t3 = (t6 + t27);
    t24 = *((unsigned char *)t3);
    t29 = (t24 == (unsigned char)3);
    if (t29 == 1)
        goto LAB104;

LAB105:    t23 = (unsigned char)0;

LAB106:    if (t23 == 1)
        goto LAB101;

LAB102:    t18 = (unsigned char)0;

LAB103:    if (t18 == 1)
        goto LAB98;

LAB99:    t17 = (unsigned char)0;

LAB100:    t4 = t17;
    goto LAB97;

LAB98:    t11 = (t0 + 1192U);
    t12 = *((char **)t11);
    t51 = (0 - 5);
    t41 = (t51 * -1);
    t52 = (1U * t41);
    t53 = (0 + t52);
    t11 = (t12 + t53);
    t43 = *((unsigned char *)t11);
    t54 = (t43 == (unsigned char)2);
    t17 = t54;
    goto LAB100;

LAB101:    t9 = (t0 + 1192U);
    t10 = *((char **)t9);
    t38 = (1 - 5);
    t34 = (t38 * -1);
    t39 = (1U * t34);
    t40 = (0 + t39);
    t9 = (t10 + t40);
    t36 = *((unsigned char *)t9);
    t42 = (t36 == (unsigned char)3);
    t18 = t42;
    goto LAB103;

LAB104:    t7 = (t0 + 1192U);
    t8 = *((char **)t7);
    t31 = (2 - 5);
    t28 = (t31 * -1);
    t32 = (1U * t28);
    t33 = (0 + t32);
    t7 = (t8 + t33);
    t30 = *((unsigned char *)t7);
    t35 = (t30 == (unsigned char)2);
    t23 = t35;
    goto LAB106;

}


extern void work_a_3044219816_3212880686_init()
{
	static char *pe[] = {(void *)work_a_3044219816_3212880686_p_0};
	xsi_register_didat("work_a_3044219816_3212880686", "isim/Test_isim_beh.exe.sim/work/a_3044219816_3212880686.didat");
	xsi_register_executes(pe);
}
