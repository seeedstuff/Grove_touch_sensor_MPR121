/*
 * MPR121_demo.ino
 * Driver for DIGITAL I2C HUMIDITY AND TEMPERATURE SENSOR
 *  
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Author     : downey
 * Create Time: May 2018
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "Seeed_MPR121_driver.h"

Mpr121 mpr121;

u8 reg_buf[128];
u16 init_filtered_data[13];

void clean_buffer(u8 *buf, u16 size)
{
  for (u16 i = 0; i<size; i++) {
    buf[i] = '\0';
  }
}

void clean_buffer(u16 *buf, u16 size)
{
  for (u16 i = 0; i<size; i++) {
    buf[i] = '\0';
  }
}

void setup()
{
  u16 line = 0;
  u8 addr = 0;
  s32 ret=0;
  Serial.begin(115200);
  if(mpr121.begin()<0)
  {
    Serial.println("Can't detect device!!!!");
  }
  else
  {
    Serial.println("mpr121 init OK!");
  }
  delay(100);
  mpr121.set_threshold(0xffff);

  // Show baseline reg
  clean_buffer(reg_buf, 13);
  for(addr = BASELINE_VALUE_REG_START_ADDR; addr < BASELINE_VALUE_REG_START_ADDR+13; addr++ ) 
  {
    mpr121.IIC_read_byte(addr, &reg_buf[addr-BASELINE_VALUE_REG_START_ADDR]);
  }
  Serial.print("Baseline:");
  for (line = 0; line < 13; line++) {
    Serial.print(" 0x");
    Serial.print(reg_buf[line], HEX);      
  }
  Serial.println();

  // show threshhold value
  clean_buffer(reg_buf, 26);  
  for(addr = THRESHOLD_REG_START_ADDR; addr < THRESHOLD_REG_START_ADDR+26; addr++ ) 
  {
    mpr121.IIC_read_byte(addr, &reg_buf[addr-THRESHOLD_REG_START_ADDR]);
  }
  Serial.print("threshold:");
  for (line = 0; line < 26; line++) {
    Serial.print(" 0x");
    Serial.print(reg_buf[line], HEX);        
  }
  Serial.println();

  // show filtered Data
  clean_buffer(init_filtered_data, 13);
  clean_buffer(reg_buf, 26);  
  for(addr = FILTERED_DATA_REG_START_ADDR_L; addr < FILTERED_DATA_REG_START_ADDR_L+26; addr++ ) 
  {
    mpr121.IIC_read_byte(addr, &reg_buf[addr-FILTERED_DATA_REG_START_ADDR_L]);
  }
  Serial.print("filtered Data:");
  for (line = 0; line < 26; line+=2) {    
    init_filtered_data[line/2] = reg_buf[line] | (reg_buf[line+1] << 8);
    Serial.print(" 0x");
    Serial.print(init_filtered_data[line/2], HEX);    
  }
  Serial.println();


  delay(2000);


  
}
void loop()
{
  u16 addr = 0;
  u16 line = 0;
  u16 filtered_data[13];
    
  // show filtered Data
  clean_buffer(reg_buf, 26);  
  for(addr = FILTERED_DATA_REG_START_ADDR_L; addr < FILTERED_DATA_REG_START_ADDR_L+26; addr++ ) 
  {
    mpr121.IIC_read_byte(addr, &reg_buf[addr-FILTERED_DATA_REG_START_ADDR_L]);
  }
  Serial.print("filtered Data delta:");
  for (line = 0; line < 26; line+=2) {
    filtered_data[line/2] = reg_buf[line] | (reg_buf[line+1] << 8);
    Serial.print("  ch");
    Serial.print(line/2);
    Serial.print(":");
    Serial.print((short)(filtered_data[line/2] - init_filtered_data[line/2]));
  }
  Serial.println();

  delay(500);

}
