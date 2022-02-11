/*
===============================================================================
 Name        : 3DGraphicsEngineProject.c
 Author      : Archita Chakraborty
 Version     : v0.0
 Copyright   : $(copyright)
 Description : 3D Graphics Engine and Diffuse Reflection
===============================================================================
*/


#include "LPC17xx.h"
#include <cr_section_macros.h>
#include "ssp.h"
#include <NXP/crp.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define PORT_NUM 0

#define ST7735_TFTWIDTH 127
#define ST7735_TFTHEIGHT 159
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29

#define swap(x, y) {x = x + y; y =x - y; x = x - y ;}

// defining color values
#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0000FF
#define RED 0xFF0000
#define WHITE 0xFFFFFF
#define GREY 0x7A7C7B
#define LEAVES 0x737000
#define BROWN 0x783F04

int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];
float lambda = 0.8;

float rotation = 0;

struct point2Dcoordinate{
	float x;
	float y;
};

float virtualCamera_x = 120.0;
float virtualCamera_y = 120.0;
float virtualCamera_z = 120.0;

float pointLightSource_x = 40.0;
float pointLightSource_y = 60.0;
float pointLightSource_z = 120.0;

float sinTheta, cosTheta, sinPhi, cosPhi, rho;


typedef struct point3DCoordinate
{
	float x;
	float y;
	float z;
} point3DCoordinate;


struct Point{

	int16_t x;
	int16_t z;

}; //used for draw tree decoration linear interpolation

void spiwrite(uint8_t c)
{
	int pnum = 0;
	src_addr[0] = c;
	SSP_SSELToggle( pnum, 0 );
	SSPSend( pnum, (uint8_t
	*)src_addr, 1 );
	SSP_SSELToggle( pnum, 1 );
}

void writecommand(uint8_t c)
{
	LPC_GPIO0->FIOCLR |= (0x1<<21);
	spiwrite(c);
}

void writedata(uint8_t c)
{
	LPC_GPIO0->FIOSET |= (0x1<<21);
	spiwrite(c);
}

void writeword(uint16_t c)
{
	uint8_t d;
	d = c >> 8;
	writedata(d);
	d = c & 0xFF;
	writedata(d);
}

void write888(uint32_t color,uint32_t repeat)
{
	uint8_t red, green, blue;

	int i;
	red = (color >> 16);
	green = (color >> 8) & 0xFF;
	blue = color & 0xFF;

	for (i = 0; i< repeat; i++) {
		writedata(red);
		writedata(green);
		writedata(blue);
	}
}

void setAddrWindow(uint16_t x0,uint16_t y0, uint16_t x1, uint16_t y1)
{
	writecommand(ST7735_CASET);
	writeword(x0);
	writeword(x1);
	writecommand(ST7735_RASET);
	writeword(y0);
	writeword(y1);
}


void lcddelay(int ms)
{
    int count = 24000;
    int i;
    for( i = count*ms; i>0; i--);
}

void lcd_init()
{
	int i;
	printf("LCD Demo Begins!!!\n");

	// Set pins P0.16, P0.21, P0.22 as output
	LPC_GPIO0->FIODIR |= (0x1<<16);
	LPC_GPIO0->FIODIR |= (0x1<<21);
	LPC_GPIO0->FIODIR |= (0x1<<22);
	// Hardware Reset Sequence
	LPC_GPIO0->FIOSET |= (0x1<<22);
	lcddelay(500);
	LPC_GPIO0->FIOCLR |= (0x1<<22);
	lcddelay(500);
	LPC_GPIO0->FIOSET |= (0x1<<22);
	lcddelay(500);
	// initialize buffers
	for ( i = 0; i < SSP_BUFSIZE; i++)
	{
	src_addr[i] = 0;
	dest_addr[i] = 0;
	}
	// Take LCD display out of sleep mode
	writecommand(ST7735_SLPOUT);
	lcddelay(200);
	// Turn LCD display on
	writecommand(ST7735_DISPON);
	lcddelay(200);
}

void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
	int16_t width, height;
	width = x1-x0+1;
	height = y1-y0+1;
	setAddrWindow(x0,y0,x1,y1);
	writecommand(ST7735_RAMWR);
	write888(color,width*height);
}

void drawPixel(int16_t x, int16_t y, uint32_t color)
{
	if ((x < 0) || (x >= _width) || (y< 0) || (y >= _height))
	return;

	setAddrWindow(x, y, x + 1, y + 1);
	writecommand(ST7735_RAMWR);
	write888(color, 1);
}


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{

    int16_t slope = abs(y1 - y0) > abs(x1 - x0);
    if (slope) {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1) {
        swap(x0, x1);
        swap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep;
    if (y0 < y1) {
        ystep = 1;
    }

    else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (slope) {
            drawPixel(y0, x0, color);
        }

        else {
            drawPixel(x0, y0, color);
        }

        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


void drawTriangleLines(float pointx0,float pointy0, float pointy1, float diffx01, float diffy01,float diffx02,float diffy02,float temp1, float temp2 ,uint32_t color)
{

	float point_x, point_y, traverse_y =pointy0;


	for(traverse_y=pointy0; traverse_y<pointy1; traverse_y++) {

		point_x = pointx0 + temp1 / diffy01;
		point_y = pointx0 + temp2 / diffy02;
		temp1 = temp1 + diffx01;
		temp2 = temp2 + diffx02;
		drawLine(point_x, traverse_y, point_y, traverse_y,color);

	}
}

void fillPolygon(int16_t pointx0, int16_t pointy0,int16_t pointx1, int16_t pointy1,int16_t pointx2, int16_t pointy2, uint32_t color)
{

	float diffx01 = pointx1 - pointx0, diffy01 = pointy1 - pointy0, diffx02 = pointx2 - pointx0, diffy02 =pointy2 - pointy0, diffx12 = pointx2 - pointx1, diffy12 = pointy2 - pointy1;

	drawTriangleLines(pointx0,pointy0, pointy1, diffx01, diffy01,diffx02,diffy02,0.0, 0.0,color);


	float  temp1=0,temp2 = diffx02 * (pointy1 - pointy0);
	float point_x, point_y, traverse_y =pointy1;

	for(; traverse_y<=pointy2; traverse_y++) {
		point_x = pointx1 + temp1 / diffy12;
		point_y = pointx0 + temp2 / diffy02;
		temp1 = temp1 + diffx12;
		temp2 = temp2 + diffx02;
		drawLine(point_x, traverse_y, point_y, traverse_y,color);
	}
}


//parameter calculation for transformation pipeline
void transformationParameterCalculation()
{
	rho=sqrt((pow(virtualCamera_x,2))+(pow(virtualCamera_y,2))+(pow(virtualCamera_z,2)));
	sinTheta = (virtualCamera_y / sqrt(pow(virtualCamera_x, 2) + pow(virtualCamera_y, 2)));
	cosTheta = (virtualCamera_x / sqrt(pow(virtualCamera_x, 2) + pow(virtualCamera_y, 2)));
	sinPhi = sqrt(pow(virtualCamera_x, 2) + pow(virtualCamera_y, 2)) / rho;
	cosPhi = (virtualCamera_z / rho);
}

//converting world coordinates to viewer coordinates by transformation pipeline and perspective projection
struct point2Dcoordinate transformationPipeline(float x_w, float y_w, float z_w)
{
	float physicalCoordinate_x, physicalCoordinate_y, focalLength=80;
	float virtual_x, virtual_y, virtual_z;

	struct point2Dcoordinate LCDDisplayCoordinates;
	transformationParameterCalculation();

	//convert world to viewer coordinates
	virtual_x = (y_w*cosTheta)-(x_w*sinTheta);
	virtual_y = (z_w*sinPhi)-(x_w*cosTheta*cosPhi)-(y_w*cosPhi*sinTheta);
	virtual_z = rho-(y_w*sinPhi*cosTheta)-(x_w*sinPhi*cosTheta)-(z_w*cosPhi);

	//Convert Viewer to 2D Coordinates
	physicalCoordinate_x = virtual_x*focalLength/virtual_z;
	physicalCoordinate_y = virtual_y*focalLength/virtual_z;

	LCDDisplayCoordinates.x = ST7735_TFTWIDTH/2 + physicalCoordinate_x  ;
	LCDDisplayCoordinates.y = ST7735_TFTHEIGHT/2 - physicalCoordinate_y ;

	return LCDDisplayCoordinates;

}

void draw3DDWorldCoordinates()
{
	struct point2Dcoordinate displayScreenCoordinates;

	float x1,y1,x2,y2,x3,y3,x4,y4;

	//Origin
	displayScreenCoordinates =transformationPipeline(0,0,0);
	x1=displayScreenCoordinates.x;
	y1=displayScreenCoordinates.y;
	//x-axis
	displayScreenCoordinates = transformationPipeline(200,0,0);
	x2=displayScreenCoordinates.x;
	y2=displayScreenCoordinates.y;
	//y-axis
	displayScreenCoordinates =transformationPipeline(0,200,0);
	x3=displayScreenCoordinates.x;
	y3=displayScreenCoordinates.y;
	//z-axis
	displayScreenCoordinates = transformationPipeline (0,0,200);
	x4=displayScreenCoordinates.x;
	y4=displayScreenCoordinates.y;

	drawLine(x1,y1,x2,y2,RED);      //x-axis denoted in red
	drawLine(x1,y1,x3,y3,GREEN);    //y-axis denoted in green
	drawLine(x1, y1, x4,y4,BLUE);   //z-axis denoted in blue

}

//shadow of cube drawn using the top surface and a point source of light
void drawCubeShadow(float x[], float y[], float z[])
{
	float shadowPoint_x[8]={0}, shadowPoint_y[8]={0}, shadowPoint_z[8]={0};
	struct point2Dcoordinate shadowPoint5,shadowPoint6,shadowPoint7,shadowPoint8;

	//n=(0,0,1) and a= (0,0,0)

	    pointLightSource_x = 10.0;
	    pointLightSource_y = 20.0;
	    pointLightSource_z = 140.0;

	for(int i=4; i<8; i++){
		float lambda= ((-1)*pointLightSource_z)/(z[i]-pointLightSource_z);

		shadowPoint_x[i]=pointLightSource_x  + lambda*(x[i]-pointLightSource_x);
		shadowPoint_y[i]=pointLightSource_y  + lambda*(y[i]-pointLightSource_y);
		shadowPoint_z[i]=pointLightSource_z  + lambda*(z[i]-pointLightSource_z);

	}

	shadowPoint5 = transformationPipeline(shadowPoint_x[4],shadowPoint_y[4],shadowPoint_z[4]);
    shadowPoint6 = transformationPipeline(shadowPoint_x[5],shadowPoint_y[5],shadowPoint_z[5]);
	shadowPoint7 = transformationPipeline(shadowPoint_x[6],shadowPoint_y[6],shadowPoint_z[6]);
	shadowPoint8 = transformationPipeline(shadowPoint_x[7],shadowPoint_y[7],shadowPoint_z[7]);

	drawLine(shadowPoint5.x, shadowPoint5.y, shadowPoint6.x,shadowPoint6.y, GREY);
	drawLine(shadowPoint6.x, shadowPoint6.y, shadowPoint7.x,shadowPoint7.y, GREY);
	drawLine(shadowPoint7.x, shadowPoint7.y, shadowPoint8.x,shadowPoint8.y, GREY);
	drawLine(shadowPoint8.x, shadowPoint8.y, shadowPoint5.x,shadowPoint5.y, GREY);

	fillPolygon(shadowPoint5.x, shadowPoint5.y,shadowPoint6.x, shadowPoint6.y, shadowPoint7.x, shadowPoint7.y,GREY);
	fillPolygon(shadowPoint5.x, shadowPoint5.y,shadowPoint8.x, shadowPoint8.y,shadowPoint7.x, shadowPoint7.y, GREY);



}

//cube structure drawn connecting the 8 points converted to viewer coordinates
void drawCube(float start_pnt, float cubeLength)
{
	struct point2Dcoordinate lcd;

	float x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7;

	virtualCamera_x = 100;
	virtualCamera_y = 100;
	virtualCamera_z = 100;

	lcd = transformationPipeline(start_pnt,start_pnt,(cubeLength+start_pnt));
	x1=lcd.x;
	y1=lcd.y;
	lcd = transformationPipeline((cubeLength+start_pnt),start_pnt,(cubeLength+start_pnt));
	x2=lcd.x;
	y2=lcd.y;
	lcd = transformationPipeline((cubeLength+start_pnt),(cubeLength+start_pnt),(cubeLength+start_pnt));
	x3=lcd.x;
	y3=lcd.y;
	lcd = transformationPipeline(start_pnt,(cubeLength+start_pnt),(cubeLength+start_pnt));
	x4=lcd.x;
	y4=lcd.y;
	lcd = transformationPipeline((cubeLength+start_pnt),start_pnt,start_pnt);
	x5=lcd.x;
	y5=lcd.y;
	lcd = transformationPipeline((cubeLength+start_pnt),(cubeLength+start_pnt),start_pnt);
	x6=lcd.x;
	y6=lcd.y;
	lcd = transformationPipeline(start_pnt,(cubeLength+start_pnt),start_pnt);
	x7=lcd.x;
	y7=lcd.y;

	drawLine(x1, y1, x2,y2,WHITE);
	drawLine(x2, y2, x3,y3,WHITE);
	drawLine(x3, y3, x4,y4,WHITE);
	drawLine(x4, y4, x1,y1,WHITE);
	drawLine(x2, y2, x5,y5,WHITE);
	drawLine(x5, y5, x6,y6,WHITE);
	drawLine(x6, y6, x3,y3,WHITE);
	drawLine(x6, y6, x7,y7,WHITE);
	drawLine(x7, y7, x4,y4,WHITE);
	lcddelay(500);
	drawLine(x4, y4, x1,y1,BLACK);

}

uint32_t calculateDiffuseColor(point3DCoordinate Point3D)
{

	struct point3DCoordinate pointSourceOfLight = {40,60,120};

	int diffusedRed, diffusedGreen, diffusedBlue;

	float r1=0.8, r2=0.0, r3=0.0,scalingFactor = 3000;

	float_t calculatedParameter = ((pointSourceOfLight.z - Point3D.z)/sqrt(pow((pointSourceOfLight.x - Point3D.x),2)+ pow((pointSourceOfLight.y - Point3D.y),2)
	+ pow((pointSourceOfLight.z - Point3D.z),2)))
	/ (pow((pointSourceOfLight.x- Point3D.x),2)
	+ pow((pointSourceOfLight.y - Point3D.y),2)
	+ pow((pointSourceOfLight.z - Point3D.z),2));

	calculatedParameter = calculatedParameter * scalingFactor;
	diffusedRed = r1 * calculatedParameter * 255;
	diffusedGreen = r2 * calculatedParameter;
	diffusedBlue = r3 * calculatedParameter;
	diffusedRed = (diffusedRed << 16);
	diffusedGreen = (diffusedGreen << 8);
	diffusedBlue = (diffusedBlue);

    return diffusedRed + diffusedGreen + diffusedBlue + 140;


}

void diffuseReflection(float cubeLength)
{

	  uint32_t diffusedColor;
     //sides of the cube filled with color
		for(int y=0;y<=cubeLength;y++){
		for(int x=0;x<=cubeLength;x++)
		{
				struct point3DCoordinate pt3D;
				struct point2Dcoordinate pt2D;
				pt3D.x=x; pt3D.y=y;pt3D.z=60;
				pt2D = transformationPipeline(pt3D.x,pt3D.y,pt3D.z);
				// RED color diffused
				diffusedColor = calculateDiffuseColor(pt3D);
				drawPixel(pt2D.x,pt2D.y,diffusedColor);
		 }
		}

		for(int y=0;y<=cubeLength;y++){
		for(int z=0;z<=cubeLength;z++)
		 {
				struct point3DCoordinate pt3D;
				struct point2Dcoordinate pt2D;
				pt3D.x=60; pt3D.y=y;pt3D.z=z;
			    pt2D = transformationPipeline(pt3D.x,pt3D.y,pt3D.z);
			    drawPixel(pt2D.x,pt2D.y,RED);
		 }
		}

		for(int z=0;z<=cubeLength;z++){
		for(int x=0;x<=cubeLength;x++)
		 {
				struct point3DCoordinate pt3D;
				struct point2Dcoordinate pt2D;
				pt3D.x=x; pt3D.y=60;pt3D.z=z;
				pt2D = transformationPipeline(pt3D.x,pt3D.y,pt3D.z);
				drawPixel(pt2D.x,pt2D.y,LIGHTBLUE);
		 }
		}

}

struct Point RotateBranches(struct Point p1, struct Point p2, int radian) {

	float angle = radian * 3.14 / 180;

	float delta_x = -1 * p1.x;

	float delta_z = -1 * p1.z;

	float translationMatrix[3][3] = {{1, 0 , delta_x}, {0, 1, delta_z}, {0, 0, 1}};

	float rotationMatrix[3][3] ={{cos(angle), -sin(angle), 0}, {sin(angle), cos(angle), 0}, {0, 0, 1}};

	float inverseTraslationMatrix[3][3] = {{1, 0, -delta_x}, {0, 1, -delta_z}, {0, 0, 1}};

	float tempMatrix[3][3] = {{0, 0 , 0}, {0, 0, 0}, {0, 0, 0}};

	// Rotation Matrix * Translation Matrix

	for (int i = 0; i < 3; i++) {

		for (int j = 0; j < 3; j++) {

			for (int k = 0; k < 3; k++){

				tempMatrix[i][j] += rotationMatrix[i][k] * translationMatrix[k][j];

			}

		}

	}

	float compositionMatrix[3][3] = {{0, 0 , 0}, {0, 0, 0}, {0, 0, 0}};

	for (int i = 0; i < 3; i++) {

		for (int j = 0; j < 3; j++) {

			for (int k = 0; k < 3; k++){

				compositionMatrix[i][j] += inverseTraslationMatrix[i][k] * tempMatrix[k][j];

			}

		}

	}


	struct Point newPoint = {0, 0};

	newPoint.x = compositionMatrix[0][0] * p2.x + compositionMatrix[0][1] * p2.z + compositionMatrix[0][2];

	newPoint.z = compositionMatrix[1][0] * p2.x + compositionMatrix[1][1] * p2.z + compositionMatrix[1][2];

	return newPoint;

}



 // finding Tip point of the branch

struct Point branchTip(struct Point point1, struct Point point2, int angle) {

	srand(time(0));

	struct Point point_temp = {0,0};

	point_temp.x = point2.x + lambda * (point2.x - point1.x);

	point_temp.z = point2.z + lambda * (point2.z - point1.z);

	struct Point newPoint = {0,0};

	newPoint = RotateBranches(point2, point_temp, angle);

	return newPoint;

}


void drawTree (struct Point trunkBottom, struct Point trunkTop, int layers,float cubeLength) {

	int arrayLen = 42;

	struct point2Dcoordinate lcd1,lcd2;

	struct Point arr[arrayLen][2];

	arr[0][0] = trunkBottom;

	arr[0][1] = trunkTop;

	int fastMovingIndex = 1;
	int slowMovingIndex = 0;

	uint32_t color;

	while (fastMovingIndex < arrayLen) {

		if (fastMovingIndex < 5) {

			color = BROWN;

		} else {

			color = LEAVES;

		}

		float radian =rand()%4 + 28; //randomizing the degree of rotation of the branches


			//drawing left tilted branches

			arr[fastMovingIndex][0] = arr[slowMovingIndex][1];

			lcd1= transformationPipeline(arr[fastMovingIndex][0].x,cubeLength,arr[fastMovingIndex][0].z);

			arr[fastMovingIndex][1] = branchTip(arr[slowMovingIndex][0], arr[slowMovingIndex][1], radian);

			lcd2= transformationPipeline(arr[fastMovingIndex][1].x,cubeLength,arr[fastMovingIndex][1].z);

			drawLine(lcd1.x,lcd1.y,lcd2.x,lcd2.y ,color);

			fastMovingIndex++;

			//drawing straight branches

			arr[fastMovingIndex][0] = arr[slowMovingIndex][1];

			lcd1= transformationPipeline(arr[fastMovingIndex][0].x,cubeLength,arr[fastMovingIndex][0].z);

			arr[fastMovingIndex][1] = branchTip(arr[slowMovingIndex][0], arr[slowMovingIndex][1], 0);

			lcd2= transformationPipeline(arr[fastMovingIndex][1].x,cubeLength,arr[fastMovingIndex][1].z);

			drawLine(lcd1.x,lcd1.y,lcd2.x,lcd2.y ,color);

			fastMovingIndex++;

			//draw right branches

			arr[fastMovingIndex][0] = arr[slowMovingIndex][1];

			lcd1= transformationPipeline(arr[fastMovingIndex][0].x,cubeLength,arr[fastMovingIndex][0].z);

			arr[fastMovingIndex][1] = branchTip(arr[slowMovingIndex][0], arr[slowMovingIndex][1], -radian);

			lcd2= transformationPipeline(arr[fastMovingIndex][1].x,cubeLength,arr[fastMovingIndex][1].z);

			drawLine(lcd1.x,lcd1.y,lcd2.x,lcd2.y ,color);

			fastMovingIndex++;

		slowMovingIndex++;

	}

}
void DecorateDrawTrees(uint32_t color,float start_pnt, float cubeLength)
{

 struct Point p0,p1;
 struct point2Dcoordinate lcd1,lcd2;

 p0.x=(cubeLength/2)+2;
 p1.x=p0.x;
 p0.z=0;
 p1.z=20;
 //p0 tree trunk bottom
 //p1 tree trunk top



   for (int i = -2; i < 3; i++) {

	   lcd1 = transformationPipeline(p0.x,cubeLength,p0.z);
	   lcd2 = transformationPipeline(p1.x,cubeLength,p1.z);

	   drawLine(lcd1.x,lcd1.y, lcd2.x, lcd2.y, BROWN);

	   p0.x=p0.x+i;
	   p1.x=p1.x+i;
		}

   drawTree (p0, p1, 5 ,cubeLength);

}



void DecorateDrawLetterA(float start_x, float start_y, float start_z, float cubeLength)
{
	float xs[5], ys[5], z = start_z + cubeLength;

	struct point2Dcoordinate p0, p1, p2, p3, p4;

	xs[0] = cubeLength/2.0;
	ys[0] = cubeLength/6.0;
	xs[1] = cubeLength - 10.0;
	ys[1] = xs[1];
	xs[2] = cubeLength/6.0;
	ys[2] = xs[1];
	xs[3] = cubeLength-20.0;
	ys[3] = (cubeLength/2.0)+5.0;
	xs[4] = cubeLength/3.0;
	ys[4] = ys[3];

	p0 = transformationPipeline(xs[0], ys[0],z);
	p1 = transformationPipeline(xs[1], ys[1],z);
	p2 = transformationPipeline(xs[2], ys[2],z);
	p3 = transformationPipeline(xs[3], ys[3],z);
	p4 = transformationPipeline(xs[4], ys[4],z);

	float m=0.0;
		while(m!=1.5)
		{
			drawLine(p0.x+m, p0.y+m, p1.x+m,p1.y+m, WHITE);
			drawLine(p0.x+m, p0.y+m, p2.x+m,p2.y+m, WHITE);
			drawLine(p4.x+m, p4.y+m, p3.x+m,p3.y+m, WHITE);
			m=m+0.25;
		}

}

void DecorateDrawLetterC(float start_x, float start_y, float start_z, float cubeLength)
{
	float zs[4], ys[4], x = start_x + cubeLength;

	struct point2Dcoordinate p0, p1, p2, p3;

	zs[0] = cubeLength - 10.0;
	ys[0] = cubeLength - 10.0;
	zs[1] = zs[0];
	ys[1] = cubeLength/3.0;
	zs[2] = cubeLength/6.0;
	ys[2] = ys[1];
	zs[3] = zs[2];
	ys[3] = zs[0];


	p0 = transformationPipeline(x, ys[0],zs[0]);
	p1 = transformationPipeline(x, ys[1],zs[1]);
	p2 = transformationPipeline(x, ys[2],zs[2]);
	p3 = transformationPipeline(x, ys[3],zs[3]);

	float m=0.0;
	while(m!=2.0)
	{
		drawLine(p0.x+m, p0.y+m, p1.x+m,p1.y+m, WHITE);
		drawLine(p1.x+m, p1.y+m, p2.x+m,p2.y+m, WHITE);
		drawLine(p2.x+m, p2.y+m, p3.x+m,p3.y+m, WHITE);
		m=m+0.25;
	}

}



int main()
{

	uint32_t pnum = PORT_NUM;
	float cubeLength = 60.0, startAtOrigin =0.0;

	//defining 8 points of the cube
	float x[8] ={startAtOrigin,(startAtOrigin+cubeLength),(startAtOrigin+cubeLength),startAtOrigin,startAtOrigin,(startAtOrigin+cubeLength),(startAtOrigin+cubeLength),startAtOrigin};
	float y[8] = {startAtOrigin,startAtOrigin, startAtOrigin+cubeLength,startAtOrigin+cubeLength, startAtOrigin,startAtOrigin, (startAtOrigin+cubeLength),(startAtOrigin+cubeLength) };
	float z[8] = {startAtOrigin,startAtOrigin, startAtOrigin, startAtOrigin,(startAtOrigin+cubeLength), (startAtOrigin+cubeLength),(startAtOrigin+cubeLength),(startAtOrigin+cubeLength)};

	if(pnum == 0)
	{
	 SSP0Init();
	}
	else
	{
		printf("Port number is not correct");
	}
	for (int i = 0; i <SSP_BUFSIZE; i++ )
	{
		src_addr[i] =
		(uint8_t)i;
		dest_addr[i] = 0;
	}

	//To initialize LCD
	lcd_init();

	//print cube points from P0 to P7, on World coordinates which have been initialized
	for (int i=0;i<8;i++)
		{
			printf("%d %f %f %f \n", i,x[i],y[i],z[i]);
		}

     //filling background with black color
	fillrect(0, 0,ST7735_TFTWIDTH, ST7735_TFTHEIGHT,BLACK);

	//drawing the world coordinate axis, xw-yw-zw
	draw3DDWorldCoordinates();
	lcddelay(500);

	//cube corners calculated already, now shadow drawn for the top 4 points
	drawCubeShadow(x, y, z ) ;

    //cube structure drawn in white frame
	drawCube(startAtOrigin,cubeLength);
		lcddelay(500);

    //diffused reflection on the top surface of the cube
	diffuseReflection(cubeLength);
	lcddelay(500);

    //Linear decoration on the 3 front visible cube surfaces
	//2D linear trees drawn on one
   DecorateDrawTrees(LEAVES,startAtOrigin,cubeLength);

	lcddelay(500);
	//Initial of my first name drawn on top surface
	DecorateDrawLetterA(0.0 ,0.0 ,0.0, cubeLength);
	lcddelay(500);
	//Initial of my second name drawn on front surface
	DecorateDrawLetterC(0.0 ,0.0 ,0.0, cubeLength);
	lcddelay(500);

	printf("LCD Demo Ends!!!\n");

	return 0;
}

