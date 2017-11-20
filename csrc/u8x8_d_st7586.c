/*

  u8x8_d_st7586.c

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2017, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  ST7586
    - has 4 different I2C addresses 
    - I2C protocol is identical to SSD13xx
  
*/


#include "u8x8.h"

/* function set, bit 2: power down, bit 3: MY, bit 4: MX, bit 5: must be 1 */
#define FS (0x020)

/* not a real power down for the ST7586... just a display off */
static const uint8_t u8x8_d_st7586_240x160_powersave0_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C( FS | 0x00 ),			/* select 00 commands */
  //U8X8_C( 0x08 ),				/* display off */
  U8X8_C( 0x0c ),				/* display on */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const uint8_t u8x8_d_st7586_240x160_powersave1_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C( FS | 0x00 ),			/* select 00 commands */
  U8X8_C( 0x08 ),				/* display off */
  //U8X8_C( 0x0c ),				/* display on */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};




static uint8_t u8x8_d_st7586_240x160_generic(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t x, c;
  uint8_t *ptr;
  switch(msg)
  {
    /* handled by the calling function
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
      u8x8_d_helper_display_setup_memory(u8x8, &u8x8_st7586_240x160_display_info);
      break;
    */
    /* handled by the calling function
    case U8X8_MSG_DISPLAY_INIT:
      u8x8_d_helper_display_init(u8x8);
      u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_240x160_init_seq);
      break;
    */
    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
      if ( arg_int == 0 )
	u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_240x160_powersave0_seq);
      else
	u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_240x160_powersave1_seq);

      /* restore orientation */
      if ( u8x8->x_offset == 0 )
	u8x8_cad_SendCmd(u8x8, FS );	/* select 00 commands */
      else
	u8x8_cad_SendCmd(u8x8, FS ^ 0x018 );	/* select 00 commands */
      
      break;
#ifdef U8X8_WITH_SET_CONTRAST
    case U8X8_MSG_DISPLAY_SET_CONTRAST:

      u8x8_cad_StartTransfer(u8x8);
      
      u8x8_cad_SendCmd(u8x8, FS );
      u8x8_cad_SendArg(u8x8, 4 | (arg_int>>7) );
      u8x8_cad_SendCmd(u8x8, FS | 1);
      u8x8_cad_SendArg(u8x8, 0x080 | arg_int );
      
      /* restore orientation */
      if ( u8x8->x_offset == 0 )
	u8x8_cad_SendCmd(u8x8, FS );	/* select 00 commands */
      else
	u8x8_cad_SendCmd(u8x8, FS ^ 0x018 );	/* select 00 commands */
      u8x8_cad_EndTransfer(u8x8);
      break;
#endif
    case U8X8_MSG_DISPLAY_DRAW_TILE:
      
      u8x8_cad_StartTransfer(u8x8);
      x = ((u8x8_tile_t *)arg_ptr)->x_pos;    
      x *= 8;
      
      x += u8x8->x_offset;
    
//      if ( u8x8->x_offset == 0 )
//	u8x8_cad_SendCmd(u8x8, FS );	/* select 00 commands */
//      else
//	u8x8_cad_SendCmd(u8x8, FS ^ 0x018 );	/* select 00 commands */
//
//      u8x8_cad_SendCmd(u8x8, 0x040 | (((u8x8_tile_t *)arg_ptr)->y_pos));
//      u8x8_cad_SendCmd(u8x8, 0x0e0 | ((x&15)));
//      u8x8_cad_SendCmd(u8x8, 0x0f0 | (x>>4) );

      u8x8_cad_SendCmd(u8x8, 0x2A); // Column Address Setting
      u8x8_cad_SendData(u8x8, 1, 0x00); // SEG0 -> SEG240
      u8x8_cad_SendData(u8x8, 1, x+8);// SEG8*3=24
      u8x8_cad_SendData(u8x8, 1, 0x00);
      u8x8_cad_SendData(u8x8, 1, 0x7f); // SEG128*3=384  seg x(dont use)  seg n  seg n
  	  u8x8_cad_SendCmd(u8x8, 0x2B); // Row Address Setting
  	  u8x8_cad_SendData(u8x8, 1, 0x00); // COM0 -> COM160
  	  u8x8_cad_SendData(u8x8, 1, ((u8x8_tile_t *)arg_ptr)->y_pos);
  	  u8x8_cad_SendData(u8x8, 1, 0x00);
  	  u8x8_cad_SendData(u8x8, 1, 0x9F);

  	  u8x8_cad_SendCmd(u8x8, 0x2C); // Write Display Data to DDRAM mode

      
      do
      {
	c = ((u8x8_tile_t *)arg_ptr)->cnt;
	ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;
	u8x8_cad_SendData(u8x8, c*8, ptr); 	/* note: SendData can not handle more than 255 bytes */
	arg_int--;
      } while( arg_int > 0 );
      
      u8x8_cad_EndTransfer(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
}

/*=============================================*/

static const u8x8_display_info_t u8x8_st7586_240x160_display_info =
{
  /* chip_enable_level = */ 0,
  /* chip_disable_level = */ 1,
  
  /* post_chip_enable_wait_ns = */ 150,
  /* pre_chip_disable_wait_ns = */ 30,
  /* reset_pulse_width_ms = */ 10, // 5
  /* post_reset_wait_ms = */ 200, // 5 		/**/
  /* sda_setup_time_ns = */ 60,		/* */
  /* sck_pulse_width_ns = */ 60,	/*  */
  /* sck_clock_hz = */ 4000000UL,	/* since Arduino 1.6.0, the SPI bus speed in Hz. Should be  1000000000/sck_pulse_width_ns */
  /* spi_mode = */ 0,		/* active high, rising edge */
  /* i2c_bus_clock_100kHz = */ 4,	/* 400KHz */
  /* data_setup_time_ns = */ 80,
  /* write_pulse_width_ns = */ 50,	
  /* tile_width = */ 16,
  /* tile_hight = */ 8,
  /* default_x_offset = */ 0,	/* must be 0, because this is checked also for normal mode */
  /* flipmode_x_offset = */ 4,		
  /* pixel_width = */ 240,
  /* pixel_height = */ 160
};



static const uint8_t u8x8_d_st7586_240x160_init_seq[] = {
    
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */

//  U8X8_C( FS | 0x03 ),			/* select 11 commands */
//  U8X8_C( 0x03 ),				/* software reset */
//
//  U8X8_C( FS | 0x00 ),			/* select 00 commands */
//  U8X8_C( 0x08 ),				/* display off */
//  //U8X8_C( 0x0c ),				/* display on */
//
//  U8X8_C( FS | 0x01 ),			/* select 01 commands */
//  U8X8_C( 0x08 ),				/* display confguration */
//  U8X8_C( 0x12 ),				/* bias 1/9 */
//  U8X8_C( 0x8f ),				/* Vop, lower 7 bits */
//
//  U8X8_C( FS | 0x00 ),			/* select 00 commands */
//  U8X8_C( 0x05),				/* Bit 0 contains high/low range for Vop */
//
//
//  U8X8_C( FS | 0x03 ),			/* select 11 commands */
//  U8X8_C( 0x0b),				/* Frame Rate: 73 Hz */
  
  U8X8_C(0x11), // Sleep Out
  U8X8_C(0x28), // Display OFF
  U8X8_DLY(50),
  U8X8_C(0xC0), // Vop = B9h
  U8X8_D1(0x3A), // Contrast_level

  U8X8_D1(0x01),
  U8X8_C(0xC3), // BIAS = 1/14
  U8X8_D1(0x00),
  U8X8_C(0xC4), // Booster = x8
  U8X8_D1(0x07),
  U8X8_C(0xD0), // Enable Analog Circuit
  U8X8_D1(0x1D),
  U8X8_C(0xB5), // N-Line = 0
  U8X8_D1(0x00),
  U8X8_C(0x39), // Monochrome Mode
  U8X8_C(0x3A), // Enable DDRAM Interface
  U8X8_D1(0x02),
  U8X8_C(0x36), // Scan Direction Setting
  U8X8_D1(0xc0),   //COM:C160--C1   SEG: SEG384-SEG1
  U8X8_C(0xB0), // Duty Setting
  U8X8_D1(0x9F),

  U8X8_C(0x20), // Display Inversion OFF
  U8X8_C(0x2A), // Column Address Setting
  U8X8_D1(0x00), // SEG0 -> SEG384
  U8X8_D1(0x00),
  U8X8_D1(0x00),
  U8X8_D1(0x7F),
  U8X8_C(0x2B), // Row Address Setting
  U8X8_D1(0x00), // COM0 -> COM160
  U8X8_D1(0x00),
  U8X8_D1(0x00),
  U8X8_D1(0x9F),

  //	display_white(); // Clear whole DDRAM by “0” (384 x 160 x 2)

  U8X8_C( 0x29), // Display ON
    
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};


static const uint8_t u8x8_d_st7586_erdbc240160_flip0_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C( FS ),					/* normal mode */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const uint8_t u8x8_d_st7586_erdbc240160_flip1_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */
  U8X8_C( FS ^ 0x018 ),					/* normal mode */
  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};


uint8_t u8x8_d_st7586_erdbc240160(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  if ( u8x8_d_st7586_240x160_generic(u8x8, msg, arg_int, arg_ptr) != 0 )
    return 1;
  if ( msg == U8X8_MSG_DISPLAY_SETUP_MEMORY )
  {
    u8x8_SetI2CAddress(u8x8, 0x07e);		/* the JLX12864 has 0x07e as a default address for I2C */
    u8x8_d_helper_display_setup_memory(u8x8, &u8x8_st7586_240x160_display_info);
    return 1;
  }
  else if ( msg == U8X8_MSG_DISPLAY_INIT )
  {
    u8x8_d_helper_display_init(u8x8);
    u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_240x160_init_seq);
    return 1;
  }
  else if  ( msg == U8X8_MSG_DISPLAY_SET_FLIP_MODE )
  {
    if ( arg_int == 0 )
    {
      u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_erdbc240160_flip0_seq);
      u8x8->x_offset = u8x8->display_info->default_x_offset;
    }
    else
    {
      u8x8_cad_SendSequence(u8x8, u8x8_d_st7586_erdbc240160_flip1_seq);
      u8x8->x_offset = u8x8->display_info->flipmode_x_offset;
    }
    return 1;
  }
  return 0;
}


