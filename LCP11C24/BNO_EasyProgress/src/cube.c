/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Gregorio Aiello 2018
 * | BNO055 libs for LPC
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "cube.h"
#include "chip.h"
#include "math.h"

#include <SSD1306.h>

//Initialize cube point arrays
double C1[] = {1,1,1};
double C2[] = {1,1,-1};
double C3[] = {1,-1,1};
double C4[] = {1,-1,-1};
double C5[] = {-1,1,1};
double C6[] = {-1,1,-1};
double C7[] = {-1,-1,1};
double C8[] = {-1,-1,-1};

double C1_home[] = {1,1,1};
double C2_home[] = {1,1,-1};
double C3_home[] = {1,-1,1};
double C4_home[] = {1,-1,-1};
double C5_home[] = {-1,1,1};
double C6_home[] = {-1,1,-1};
double C7_home[] = {-1,-1,1};
double C8_home[] = {-1,-1,-1};

int scale = 16, sZ = 4; //Overall scale and perspective distance
double calX = -36.78, calY = -52.8, calZ = 7.24;
double rotX, rotY, rotZ;

static void vectRotX(double angle){  //Rotates all the points around the x axis.
  double yt = C1_home[1];
  double zt = C1_home[2];
  C1[1] = yt*cos(angle)-zt*sin(angle);
  C1[2] = yt*sin(angle)+zt*cos(angle);

  yt = C2_home[1];
  zt = C2_home[2];
  C2[1] = yt*cos(angle)-zt*sin(angle);
  C2[2] = yt*sin(angle)+zt*cos(angle);

  yt = C3_home[1];
  zt = C3_home[2];
  C3[1] = yt*cos(angle)-zt*sin(angle);
  C3[2] = yt*sin(angle)+zt*cos(angle);

  yt = C4_home[1];
  zt = C4_home[2];
  C4[1] = yt*cos(angle)-zt*sin(angle);
  C4[2] = yt*sin(angle)+zt*cos(angle);

  yt = C5_home[1];
  zt = C5_home[2];
  C5[1] = yt*cos(angle)-zt*sin(angle);
  C5[2] = yt*sin(angle)+zt*cos(angle);

  yt = C6_home[1];
  zt = C6_home[2];
  C6[1] = yt*cos(angle)-zt*sin(angle);
  C6[2] = yt*sin(angle)+zt*cos(angle);

  yt = C7_home[1];
  zt = C7_home[2];
  C7[1] = yt*cos(angle)-zt*sin(angle);
  C7[2] = yt*sin(angle)+zt*cos(angle);

  yt = C8_home[1];
  zt = C8_home[2];
  C8[1] = yt*cos(angle)-zt*sin(angle);
  C8[2] = yt*sin(angle)+zt*cos(angle);
}

static void vectRotY(double angle){  //Rotates all the points around the y axis.
  double xt = C1_home[0];
  double zt = C1[2];
  C1[0] = xt*cos(angle)+zt*sin(angle);
  C1[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C2_home[0];
  zt = C2[2];
  C2[0] = xt*cos(angle)+zt*sin(angle);
  C2[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C3_home[0];
  zt = C3[2];
  C3[0] = xt*cos(angle)+zt*sin(angle);
  C3[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C4_home[0];
  zt = C4[2];
  C4[0] = xt*cos(angle)+zt*sin(angle);
  C4[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C5_home[0];
  zt = C5[2];
  C5[0] = xt*cos(angle)+zt*sin(angle);
  C5[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C6_home[0];
  zt = C6[2];
  C6[0] = xt*cos(angle)+zt*sin(angle);
  C6[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C7_home[0];
  zt = C7[2];
  C7[0] = xt*cos(angle)+zt*sin(angle);
  C7[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C8_home[0];
  zt = C8[2];
  C8[0] = xt*cos(angle)+zt*sin(angle);
  C8[2] = -xt*sin(angle)+zt*cos(angle);

}

static void vectRotZ(double angle){  //Rotates all the points around the z axis.
  double xt = C1[0];
  double yt = C1[1];
  C1[0] = xt*cos(angle)+yt*sin(angle);
  C1[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C2[0];
  yt = C2[1];
  C2[0] = xt*cos(angle)+yt*sin(angle);
  C2[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C3[0];
  yt = C3[1];
  C3[0] = xt*cos(angle)+yt*sin(angle);
  C3[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C4[0];
  yt = C4[1];
  C4[0] = xt*cos(angle)+yt*sin(angle);
  C4[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C5[0];
  yt = C5[1];
  C5[0] = xt*cos(angle)+yt*sin(angle);
  C5[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C6[0];
  yt = C6[1];
  C6[0] = xt*cos(angle)+yt*sin(angle);
  C6[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C7[0];
  yt = C7[1];
  C7[0] = xt*cos(angle)+yt*sin(angle);
  C7[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C8[0];
  yt = C8[1];
  C8[0] = xt*cos(angle)+yt*sin(angle);
  C8[1] = -xt*sin(angle)+yt*cos(angle);
}

void cube(double *Euler){
	  vectRotX(Euler[2]);
	  vectRotY(Euler[0]);
	  vectRotZ(Euler[1]);

	  SSD1306_Fill(SSD1306_COLOR_WHITE);
	  SSD1306_DrawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1],SSD1306_COLOR_BLACK); //1-2
	  SSD1306_DrawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1],SSD1306_COLOR_BLACK); //1-3
	  SSD1306_DrawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1],SSD1306_COLOR_BLACK); //1-5
	  SSD1306_DrawLine(64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1], 64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1],SSD1306_COLOR_BLACK); //2-4
	  SSD1306_DrawLine(64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1], 64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1],SSD1306_COLOR_BLACK); //2-6
	  SSD1306_DrawLine(64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1], 64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1],SSD1306_COLOR_BLACK); //3-4
	  SSD1306_DrawLine(64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1], 64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1],SSD1306_COLOR_BLACK); //3-7
	  SSD1306_DrawLine(64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1],SSD1306_COLOR_BLACK); //4-8
	  SSD1306_DrawLine(64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1], 64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1],SSD1306_COLOR_BLACK); //5-6
	  SSD1306_DrawLine(64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1], 64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1],SSD1306_COLOR_BLACK); //5-7
	  SSD1306_DrawLine(64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1],SSD1306_COLOR_BLACK); //6-8
	  SSD1306_DrawLine(64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1],SSD1306_COLOR_BLACK); //7-8
	  SSD1306_UpdateOLED();
}
