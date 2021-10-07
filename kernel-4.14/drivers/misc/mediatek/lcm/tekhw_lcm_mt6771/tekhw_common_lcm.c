#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include "lcm_drv.h"
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>
#endif
#include "lcm_drv.h"



#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#include <lcm_pmic.h>
#ifdef MTK_ROUND_CORNER_SUPPORT
#include "data_rgba8888_roundedpattern.h"
#endif
#else
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "data_rgba8888_roundedpattern.h"
#endif
#endif


#include <teksunhw.h>/*add by xunhu andy andy20160710 at 23:09*/
#include "tekhw_common_lcm_gpio.h"
extern tekhw_lcd_data* tekhw_current_lcd_data_ptr;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int display_bias_enable(void);
extern int display_bias_disable(void);
#ifdef BUILD_LK
static LCM_UTIL_FUNCS lcm_util = {0};
#else
static struct LCM_UTIL_FUNCS lcm_util = {0};
#endif
#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg                                          lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifdef BUILD_LK
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util);
static void lcm_get_params(LCM_PARAMS *params);
#else
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util);
static void lcm_get_params(struct LCM_PARAMS *params);
#endif

static void lcm_init(void);
static void lcm_suspend(void);
static void lcm_resume(void);
static unsigned int lcm_compare_id(void);
static tekhw_u16 temp=-1;

static  int suspend_reset=-1;
static int gpio_power_setting[2]={-1};
static int gpio_id_pin[3]={-1};
static int read_id_type[6]={-1};
static int j=0;
static tekhw_u16 tekhw_send_read_cmd(const char *table, unsigned int count, unsigned char force_update)
{
	unsigned int j;
    tekhw_u16 ret = 0,id = 0,size = 0, readid_first = 1;
    unsigned int array[64];
    tekhw_u16* ptr = (tekhw_u16*)table;
    char  buffer[5],buffer1[5];
    char para_list[180];
    //TEKHW_TRACE("tekhw_send_read_cmd start table=0x%x,count=0x%x\n",(unsigned int)table,(unsigned int)count);

    while(((char *)ptr - table) < count)
    {
        //TEKHW_TRACE("cmd = 0x%x,ptr=0x%x\n",*ptr,ptr);
        switch (*ptr++)
        {
            case TEKHW_DSI_SET_CMDQ :
                size = *ptr++;
                for(j=0; j<size/sizeof(tekhw_u32); j++)
                {
                    *(array+j) = (*ptr<<24)|(*(ptr+1)<<16)|(*(ptr+2)<<8)|(*(ptr+3));
                    ptr += 4;
                    //TEKHW_TRACE("TEKHW_DSI_SET_CMDQ array[%d] =0x%x\n",j, (unsigned int)*(array+j));
                }
                dsi_set_cmdq(array, j, 1);
                break;

            case TEKHW_DSI_SET_CMDQ_V2:
                for(j=0; j < *(ptr + 1) ; j++)
                {
                    //TEKHW_TRACE("TEKHW_DSI_SET_CMDQ_V2:cmd=0x%x, data[%d]=0x%x\n",(unsigned int)*ptr, j, (unsigned int)*(ptr+2+j));
                    para_list[j] = *(ptr+2+j);
                }
                dsi_set_cmdq_V2(*ptr, *(ptr+1), para_list, force_update);
                ptr += *(ptr + 1) + 2;
                break;

            case TEKHW_DSI_WRITE_CMD:
                /*æ²¡è§?è¿?*/
                break;

            case TEKHW_DSI_WRITE_REGS:
                /*æ²¡è§?è¿?*/
                break;

            case TEKHW_LCD_DELAY_MS:
                //TEKHW_TRACE("TEKHW_LCD_DELAY_MS = 0x%x\n",(unsigned int)*ptr);
                MDELAY(*ptr++);
                break;

            case TEKHW_DSI_DCS_READ_LCM_REG_V2:
            #if 0//removed by xunhu andy andy20160716 at 09:47
                for(j=0; j < *ptr; j++)
                {
                    TEKHW_TRACE("TEKHW_DSI_DCS_READ_LCM_REG_V2[%d] =0x%x\n",j, *(ptr+j));
                }
            #endif
                id = read_reg_v2(*(ptr+1), buffer, *(ptr+2));
                ptr += *ptr + 1;

					if(read_id_type[0]==0)
					{
						ret = (buffer[0]<<8)|buffer[1];
					}
					
					else if(read_id_type[0]==1)
					{
						ret = (buffer[2]<<8)|buffer[3];
					}
					else if(read_id_type[0]==2)
					{
						if(readid_first)
						{
							ret = buffer[0];
							readid_first = 0;
						}
						else
						{
							ret = (ret<<8) | buffer[0];
						}
					}
					else if(read_id_type[0]==3)
					{
						if(readid_first)
						{
							ret = buffer[1];
							readid_first = 0;
						}
						else
						{
							ret = (ret<<8) | buffer[1];
						}
					}
					else if(read_id_type[0]==4)
					{
						if(read_id_type[1]==1)
						{
								if(read_id_type[3]==0)
								{
										ret = buffer[read_id_type[2]];
										TEKHW_TRACE("xunhu--------read id=%x----------------------\n",ret);
									}
								else
								{
									ret = (buffer[read_id_type[2]]<<8|buffer[read_id_type[3]]);
									TEKHW_TRACE("xunhu--------read id=%x----------------------\n",ret);
								}
						}
						else if(read_id_type[1]==2)
						{
								if(readid_first)
								{
									memcpy(buffer1,buffer,sizeof(buffer));
									readid_first = 0;
									
								}
								else
								{
									ret = (buffer1[read_id_type[2]]<<8) | buffer[read_id_type[3]];
									TEKHW_TRACE("xunhu--------read id=%x----------------------\n",ret);
								}							
							}
					}
				break;
            case TEKHW_LCD_RESET_CMD:
                TEKHW_TRACE("TEKHW_LCM_RESET_CMD = 0x%x\n",(unsigned int)*ptr);
                temp=*ptr++;
                SET_RESET_PIN(temp);
				#if 0 //ndef BUILD_LK
                if(suspend_reset==1)
                {
					TEKHW_TRACE("xunhu----------lcm=%d-------temp---\n",temp);
					if(temp)
						lcm_set_rst_output(LCM_RST_GPIO_HIGH);
					else
						lcm_set_rst_output(LCM_RST_GPIO_LOW);
				}
				else {
					lcm_set_rst_output(LCM_RST_MODE);
				}
				#endif
                break;
            default:
                TEKHW_TRACE("can not run here \n");
                TEKHW_ASSERT(0);
                break;
       	}

        //TEKHW_TRACE("after cmd = 0x%x,ptr=0x%x\n",*ptr,ptr);
    }
	return ret;
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
#else
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}
#endif
#ifdef BUILD_LK
static void lcm_get_params(LCM_PARAMS *params)
#else
static void lcm_get_params(struct LCM_PARAMS *params)
#endif
{
	int i,j,t=0;
    TEKHW_TRACE("lcm_get_params start\n");
    TEKHW_ASSERT(tekhw_current_lcd_data_ptr != 0);
    TEKHW_ASSERT(tekhw_current_lcd_data_ptr->name.size > 0);
#ifdef BUILD_LK
    memset(params, 0, sizeof(LCM_PARAMS));
#else
    memset(params, 0, sizeof(struct LCM_PARAMS));
#endif
#ifdef BUILD_LK
#ifdef MTK_ROUND_CORNER_SUPPORT
params->round_corner_en = 1;
params->round_corner_params.w = ROUND_CORNER_W;
params->round_corner_params.h = ROUND_CORNER_H;
params->round_corner_params.lt_addr = left_top;
params->round_corner_params.rt_addr = right_top;
params->round_corner_params.lb_addr = left_bottom;
params->round_corner_params.rb_addr = right_bottom;
#endif
#else
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
params->round_corner_en = 1;
params->corner_pattern_width =32;//lcm round width
params->corner_pattern_height =32;//lcm round height
#endif
#endif

    params->type   = (tekhw_current_lcd_data_ptr->type.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->type.buffer) : 0;

    params->width  = (tekhw_current_lcd_data_ptr->width.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->width.buffer) : 0;
    params->height = (tekhw_current_lcd_data_ptr->height.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->height.buffer) : 0;
    params->physical_width = (tekhw_current_lcd_data_ptr->physical_width.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->physical_width.buffer) : 0;
    params->physical_height = (tekhw_current_lcd_data_ptr->physical_height.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->physical_height.buffer) : 0;
    // enable tearing-free
    params->dbi.te_mode 				= (tekhw_current_lcd_data_ptr->te_mode.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->te_mode.buffer) : 0;  //LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= (tekhw_current_lcd_data_ptr->te_edge_polarity.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->te_edge_polarity.buffer) : 0;

    params->dsi.mode   = (tekhw_current_lcd_data_ptr->mode.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->mode.buffer) : 0;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= (tekhw_current_lcd_data_ptr->lane_num.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->lane_num.buffer) : 0;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = (tekhw_current_lcd_data_ptr->color_order.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->color_order.buffer) : 0;
    params->dsi.data_format.trans_seq   = (tekhw_current_lcd_data_ptr->trans_seq.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->trans_seq.buffer) : 0;
    params->dsi.data_format.padding     = (tekhw_current_lcd_data_ptr->padding.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->padding.buffer) : 0;
    params->dsi.data_format.format      = (tekhw_current_lcd_data_ptr->format.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->format.buffer) : 0;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=(tekhw_current_lcd_data_ptr->packet_size.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->packet_size.buffer) : 0;

    // Video mode setting
    //params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS=(tekhw_current_lcd_data_ptr->ps.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->ps.buffer) : 0;

    params->dsi.vertical_sync_active				= (tekhw_current_lcd_data_ptr->vertical_sync_active.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->vertical_sync_active.buffer) : 0;//2;
    params->dsi.vertical_backporch					= (tekhw_current_lcd_data_ptr->vertical_backporch.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->vertical_backporch.buffer) : 0;  // from Q driver
    params->dsi.vertical_frontporch					= (tekhw_current_lcd_data_ptr->vertical_frontporch.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->vertical_frontporch.buffer) : 0;  // rom Q driver
    params->dsi.vertical_active_line				= params->height;

    params->dsi.horizontal_sync_active				= (tekhw_current_lcd_data_ptr->horizontal_sync_active.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->horizontal_sync_active.buffer) : 0;//10;
    params->dsi.horizontal_backporch				= (tekhw_current_lcd_data_ptr->horizontal_backporch.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->horizontal_backporch.buffer) : 0; // from Q driver
    params->dsi.horizontal_frontporch				= (tekhw_current_lcd_data_ptr->horizontal_frontporch.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->horizontal_frontporch.buffer) : 0;
    params->dsi.horizontal_active_pixel				= params->width;

    params->dsi.PLL_CLOCK = (tekhw_current_lcd_data_ptr->pll_clock.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->pll_clock.buffer) : 0;//240; //this value must be in MTK suggested table
    params->dsi.ssc_disable					= (tekhw_current_lcd_data_ptr->ssc_disable.size > 0) ? *((tekhw_u16*)tekhw_current_lcd_data_ptr->ssc_disable.buffer) : 0;
	if(tekhw_current_lcd_data_ptr->need_esd_check.size > 0)
	{
		if(tekhw_current_lcd_data_ptr->noncont_clock.size > 0)
			params->dsi.noncont_clock=*((tekhw_u16*)tekhw_current_lcd_data_ptr->noncont_clock.buffer);
		if(tekhw_current_lcd_data_ptr->noncont_clock_period.size > 0)
			params->dsi.noncont_clock_period=*((tekhw_u16*)tekhw_current_lcd_data_ptr->noncont_clock_period.buffer);
		params->dsi.esd_check_enable						= 1;
		params->dsi.customization_esd_check_enable 			= 1;
		for(i=0;i<*((tekhw_u16*)tekhw_current_lcd_data_ptr->need_esd_check.buffer + 0);i++)
		{
			if(t<tekhw_current_lcd_data_ptr->need_esd_check.size)
			{
					params->dsi.lcm_esd_check_table[i].cmd				= *((tekhw_u16*)tekhw_current_lcd_data_ptr->need_esd_check.buffer +t+ 1);
					params->dsi.lcm_esd_check_table[i].count			= *((tekhw_u16*)tekhw_current_lcd_data_ptr->need_esd_check.buffer +t+ 2);
					for(j=0;j<params->dsi.lcm_esd_check_table[i].count;j++)
					{
							params->dsi.lcm_esd_check_table[i].para_list[j]		= *((tekhw_u16*)tekhw_current_lcd_data_ptr->need_esd_check.buffer +t+ 3+j);
					}
					t=t+params->dsi.lcm_esd_check_table[i].count+2;
			}
		}
		
}

#if 0//removed by xunhu andy andy20160716 at 09:48
    TEKHW_TRACE("lcm_get_params params->type =%d\n",params->type);
    TEKHW_TRACE("lcm_get_params params->width =%d\n",params->width);
    TEKHW_TRACE("lcm_get_params params->height =%d\n",params->height);
    TEKHW_TRACE("lcm_get_params params->dbi.te_mode  =%d\n",params->dbi.te_mode );
    TEKHW_TRACE("lcm_get_params params->dbi.te_edge_polarity =%d\n",params->dbi.te_edge_polarity);
    TEKHW_TRACE("lcm_get_params params->dsi.mode =%d\n",params->dsi.mode);
    TEKHW_TRACE("lcm_get_params params->dsi.LANE_NUM =%d\n",params->dsi.LANE_NUM);
    TEKHW_TRACE("lcm_get_params params->dsi.data_format.color_order =%d\n",params->dsi.data_format.color_order);
    TEKHW_TRACE("lcm_get_params params->dsi.data_format.trans_seq =%d\n",params->dsi.data_format.trans_seq);
    TEKHW_TRACE("lcm_get_params params->dsi.data_format.padding =%d\n",params->dsi.data_format.padding);
    TEKHW_TRACE("lcm_get_params params->dsi.data_format.format =%d\n",params->dsi.data_format.format);
    TEKHW_TRACE("lcm_get_params params->dsi.packet_size =%d\n",params->dsi.packet_size);
    TEKHW_TRACE("lcm_get_params params->dsi.PS =%d\n",params->dsi.PS);
    TEKHW_TRACE("lcm_get_params params->dsi.vertical_sync_active =%d\n",params->dsi.vertical_sync_active);
    TEKHW_TRACE("lcm_get_params params->dsi.vertical_backporch =%d\n",params->dsi.vertical_backporch);
    TEKHW_TRACE("lcm_get_params params->dsi.vertical_frontporch =%d\n",params->dsi.vertical_frontporch);
    TEKHW_TRACE("lcm_get_params params->dsi.vertical_active_line =%d\n",params->dsi.vertical_active_line);
    TEKHW_TRACE("lcm_get_params params->horizontal_sync_active =%d\n",params->dsi.horizontal_sync_active);
    TEKHW_TRACE("lcm_get_params params->dsi.horizontal_backporch =%d\n",params->dsi.horizontal_backporch);
    TEKHW_TRACE("lcm_get_params params->dsi.horizontal_frontporch =%d\n",params->dsi.horizontal_frontporch);
    TEKHW_TRACE("lcm_get_params params->dsi.horizontal_active_pixel=%d\n",params->dsi.horizontal_active_pixel);
    TEKHW_TRACE("lcm_get_params params->dsi.PLL_CLOCK=%d\n",params->dsi.PLL_CLOCK );
    TEKHW_TRACE("lcm_get_params params->dsi.ssc_disable =%d\n",params->dsi.ssc_disable);
#endif
}

static void lcm_display_bias_set(int en)
{
	#ifdef BUILD_LK	
	int ret = 0;
	#endif
	if(en==1)
	{
		#ifdef BUILD_LK	
		/*config rt5081 register 0xB2[7:6]=0x3, that is set db_delay=4ms.*/
		ret = PMU_REG_MASK(0xB2, (0x3 << 6), (0x3 << 6));
		/* set AVDD 5.4v, (4v+28*0.05v) */
		/*ret = RT5081_write_byte(0xB3, (1 << 6) | 28);*/
		ret = PMU_REG_MASK(0xB3, 36, (0x3F << 0));
		/* set AVEE */
		/*ret = RT5081_write_byte(0xB4, (1 << 6) | 28);*/
		ret = PMU_REG_MASK(0xB4, 36, (0x3F << 0));
		/* enable AVDD & AVEE */
		/* 0x12--default value; bit3--Vneg; bit6--Vpos; */
		/*ret = RT5081_write_byte(0xB1, 0x12 | (1<<3) | (1<<6));*/
		ret = PMU_REG_MASK(0xB1, (1<<3) | (1<<6), (1<<3) | (1<<6));
		MDELAY(15);
		#else
		display_bias_enable();
		#endif
	}
	else
	{
		#ifdef BUILD_LK	
			/* enable AVDD & AVEE */
			/* 0x12--default value; bit3--Vneg; bit6--Vpos; */
			/*ret = RT5081_write_byte(0xB1, 0x12);*/
			ret = PMU_REG_MASK(0xB1, (0<<3) | (0<<6), (1<<3) | (1<<6));
			MDELAY(5);
		#else
			display_bias_disable();
		#endif		
	}
}
static void lcm_init(void)
{
      if (tekhw_current_lcd_data_ptr->gpio_power.size >=2)
      {
	  		for(j=0;j<tekhw_current_lcd_data_ptr->gpio_power.size;j++)
			{
				gpio_power_setting[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->gpio_power.buffer+j);
				TEKHW_TRACE("xunhu-----------gpio_power_setting[%d]=%d-------------------------\n",j,gpio_power_setting[j]);
			}
			if(gpio_power_setting[0] >=200)
			{
				lcm_display_bias_set(1);
			}
			else
			{
				lcm_set_gpio_output(gpio_power_setting[0],1);
				lcm_set_gpio_output(gpio_power_setting[1],1);			
			}
	}
	
	if(read_id_type[0]<0)
	{
		if (tekhw_current_lcd_data_ptr->read_id_mask.size > 0)
		{
			for(j=0;j</*tekhw_current_lcd_data_ptr->read_id_mask.size*/4;j++)
			{
					read_id_type[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->read_id_mask.buffer+j);
					TEKHW_TRACE("xunhu-----------read_id_type[%d]=%d-------------------------\n",j,read_id_type[j]);
			}
		}
	}
    if(tekhw_current_lcd_data_ptr->init.size > 0)
    {
        TEKHW_TRACE("%s lcm_init start\n",tekhw_current_lcd_data_ptr->name.buffer);
        tekhw_send_read_cmd(tekhw_current_lcd_data_ptr->init.buffer, tekhw_current_lcd_data_ptr->init.size, 1);
    }
    else
    {
        TEKHW_TRACE("%s lcm_init init.size = 0\n",tekhw_current_lcd_data_ptr->name.buffer);
    }
}

static void lcm_suspend(void)
{

	suspend_reset=1;
    if(tekhw_current_lcd_data_ptr->enter_sleep.size > 0)
    {
        TEKHW_TRACE("%s lcm_suspend start\n",tekhw_current_lcd_data_ptr->name.buffer);
        tekhw_send_read_cmd(tekhw_current_lcd_data_ptr->enter_sleep.buffer, tekhw_current_lcd_data_ptr->enter_sleep.size, 1);
    }

    if (tekhw_current_lcd_data_ptr->gpio_power.size >=2)
      {
	  		for(j=0;j<tekhw_current_lcd_data_ptr->gpio_power.size;j++)
			{
				gpio_power_setting[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->gpio_power.buffer+j);
				TEKHW_TRACE("xunhu-----------gpio_power_setting[%d]=%d-------------------------\n",j,gpio_power_setting[j]);
			}
			if(gpio_power_setting[0] >=200)
			{
				lcm_display_bias_set(0);
			}
			else
			{
				lcm_set_gpio_output(gpio_power_setting[0],1);
				lcm_set_gpio_output(gpio_power_setting[1],1);			
			}
	}
    else
    {
        TEKHW_TRACE("%s lcm_suspend enter_sleep.size = 0\n",tekhw_current_lcd_data_ptr->name.buffer);
    }

    suspend_reset=0;

}

static void lcm_resume(void)
{

    if(tekhw_current_lcd_data_ptr->exist_sleep.size > 0)
    {
	 if (tekhw_current_lcd_data_ptr->gpio_power.size >=2)
      {
	  		for(j=0;j<tekhw_current_lcd_data_ptr->gpio_power.size/2;j++)
			{
				gpio_power_setting[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->gpio_power.buffer+j);
				TEKHW_TRACE("xunhu-----------gpio_power_setting[%d]=%d-------------------------\n",j,gpio_power_setting[j]);
			}
			if(gpio_power_setting[0] >=200)
			{
				lcm_display_bias_set(1);
			}
			else
			{
				lcm_set_gpio_output(gpio_power_setting[0],1);
				lcm_set_gpio_output(gpio_power_setting[1],1);			
			}
	     }
        TEKHW_TRACE("%s lcm_resume start\n",tekhw_current_lcd_data_ptr->name.buffer);
        tekhw_send_read_cmd(tekhw_current_lcd_data_ptr->exist_sleep.buffer, tekhw_current_lcd_data_ptr->exist_sleep.size, 1);
    }
    else
    {
        TEKHW_TRACE("%s lcm_suspend exist_sleep.size = 0,replace init code to exit sleep\n",tekhw_current_lcd_data_ptr->name.buffer);
        lcm_init();
    }

}


static unsigned int lcm_compare_id(void)
{
    tekhw_u16 id=0,j=0;
    int rawdata = 0,adc_voltage_id=0,res;
    int data[4] = {0,0,0,0};
    TEKHW_TRACE("%s lcm_compare_id start\n",tekhw_current_lcd_data_ptr->name.buffer);
     if (tekhw_current_lcd_data_ptr->gpio_power.size >=2)
      {
	  		for(j=0;j<tekhw_current_lcd_data_ptr->gpio_power.size/2;j++)
			{
				gpio_power_setting[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->gpio_power.buffer+j);
				TEKHW_TRACE("xunhu-----------gpio_power_setting[%d]=%d-------------------------\n",j,gpio_power_setting[j]);
			}
			if(gpio_power_setting[0] >=200)
			{
				lcm_display_bias_set(1);
			}
			else
			{
				lcm_set_gpio_output(gpio_power_setting[0],1);
				lcm_set_gpio_output(gpio_power_setting[1],1);			
			}
	}
	if (tekhw_current_lcd_data_ptr->read_id_mask.size > 0)
		{
			for(j=0;j<tekhw_current_lcd_data_ptr->read_id_mask.size/2;j++)
			{
					read_id_type[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->read_id_mask.buffer+j);
					TEKHW_TRACE("xunhu-----------read_id_type[%d]=%d-------------------------\n",j,read_id_type[j]);
			}
		}
    if(tekhw_current_lcd_data_ptr->read_id.size > 0)
    {
        id = tekhw_send_read_cmd(tekhw_current_lcd_data_ptr->read_id.buffer, tekhw_current_lcd_data_ptr->read_id.size, 1);
    }

    if (tekhw_current_lcd_data_ptr->lcd_id.size > 0)
    {
        TEKHW_TRACE("%s read id=0x%x, tekhw_current_lcd_data_ptr->lcm_id = 0x%x\n", tekhw_current_lcd_data_ptr->name.buffer, (unsigned int)id, (unsigned int)*((tekhw_u16*)tekhw_current_lcd_data_ptr->lcd_id.buffer));
    	if (*((tekhw_u16*)tekhw_current_lcd_data_ptr->lcd_id.buffer) == id)
    	{
			if (tekhw_current_lcd_data_ptr->gpio_id_pin.size >=3)
      		{
				for(j=0;j<3;j++)
				{
					gpio_id_pin[j]=*((tekhw_u16*)tekhw_current_lcd_data_ptr->gpio_id_pin.buffer+j);
					TEKHW_TRACE("xunhu-----lcd  gpio_id_pin[%d]=%d\n",j,gpio_id_pin[j]);
				}
				if(gpio_id_pin[0]==0)
				{
					if(lcm_get_gpio_value(gpio_id_pin[1]) == gpio_id_pin[2])
					{
						return 1;
					}
					else
					{
						return 0;
					}
				}
				else if(gpio_id_pin[0]==1)
				{
					res = IMM_GetOneChannelValue(gpio_id_pin[1],data,&rawdata);
					adc_voltage_id = data[0]*1000+data[1]*10;
					TEKHW_TRACE("xunhu-----lcd  adc id=%d\n",adc_voltage_id);
					if(adc_voltage_id<1000)
					{
							adc_voltage_id=1000;
					}
					else if(adc_voltage_id>1000&&adc_voltage_id<2000)
					{
							adc_voltage_id=2000;
					}
					else if(adc_voltage_id>2000&&adc_voltage_id<3000)
					{
							adc_voltage_id=3000;
					}
					else if(adc_voltage_id>3000&&adc_voltage_id<4000)
					{
							adc_voltage_id=4000;
					}
					if(adc_voltage_id==gpio_id_pin[2])
					{
							return 1;
					}
					else
					{
							return 0;
					}
				}
			}
			else
			{
					return 1;
				}
    	}
    	else
    	{
    		return 0;
    	}
    }

    TEKHW_TRACE("%s lcm_compare_id force return 1\n",tekhw_current_lcd_data_ptr->name.buffer);
    return 1;
}
#ifdef BUILD_LK
LCM_DRIVER tekhw_common_lcm_drv =
#else
struct LCM_DRIVER tekhw_common_lcm_drv =
#endif
{
    .name			= "tekhw_lcd_name",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};


