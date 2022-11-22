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

#include "xsi.h"

struct XSI_INFO xsi_info;

char *IEEE_P_3499444699;
char *IEEE_P_3620187407;
char *IEEE_P_2592010699;
char *IEEE_P_1242562249;
char *STD_STANDARD;


int main(int argc, char **argv)
{
    xsi_init_design(argc, argv);
    xsi_register_info(&xsi_info);

    xsi_register_min_prec_unit(-12);
    ieee_p_2592010699_init();
    ieee_p_3499444699_init();
    ieee_p_3620187407_init();
    ieee_p_1242562249_init();
    work_a_2354835776_3212880686_init();
    work_a_4266567250_3212880686_init();
    work_a_2252756781_3212880686_init();
    work_a_4100369738_3212880686_init();
    work_a_0181165716_3212880686_init();
    work_a_0678885791_3212880686_init();
    work_a_2166523021_3212880686_init();
    work_a_3044219816_3212880686_init();
    work_a_3282054447_3212880686_init();
    work_a_1682115572_3212880686_init();
    work_a_3222946569_3212880686_init();
    work_a_1489802966_3212880686_init();
    work_a_1580005921_3212880686_init();
    work_a_2696640741_3212880686_init();
    work_a_3421264124_2372691052_init();


    xsi_register_tops("work_a_3421264124_2372691052");

    IEEE_P_3499444699 = xsi_get_engine_memory("ieee_p_3499444699");
    IEEE_P_3620187407 = xsi_get_engine_memory("ieee_p_3620187407");
    IEEE_P_2592010699 = xsi_get_engine_memory("ieee_p_2592010699");
    xsi_register_ieee_std_logic_1164(IEEE_P_2592010699);
    IEEE_P_1242562249 = xsi_get_engine_memory("ieee_p_1242562249");
    STD_STANDARD = xsi_get_engine_memory("std_standard");

    return xsi_run_simulation(argc, argv);

}
